#![deny(unsafe_code)]
#![deny(warnings)]
#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]

use panic_halt as _;
mod kalman;

#[rtic::app(device = stm32f4xx_hal::pac, peripherals = true, dispatchers = [SPI1, SPI2])]
mod app {
    use core::f32::consts::PI;
    use heapless::spsc::{Consumer, Producer, Queue};
    use rtic_monotonics::{create_systick_token, systick::Systick};
    use stm32f4xx_hal::{self as hal, pac::TIM5};

    use hal::{
        dma::{config::DmaConfig, DmaFlag, PeripheralToMemory, Stream2, StreamsTuple, Transfer}, 
        i2c::{I2c1, Mode}, 
        pac::{DMA2, TIM4, USART1}, 
        prelude::*, rcc::RccExt, serial, 
        timer::{self, Event}
    };

    use rtt_target::{rprintln, rtt_init_print};

    /** Kalman Filter Libraries **/
    use libm::{atanf, sqrtf};
    use crate::kalman::KalmanFilter;
    use lsm6dsox_driver::Lsm6dsox;


    const BUFFER_SIZE: usize = 100;

    type RxTransfer = Transfer<
        Stream2<DMA2>,
        4,
        serial::Rx<USART1>,
        PeripheralToMemory,
        &'static mut [u8; BUFFER_SIZE],
    >;

    #[shared]
    struct Shared {
        #[lock_free]
        rx_transfer: RxTransfer,
    }

    #[local]
    struct Local {
        /** FlySky Buffer **/
        rx_buffer: Option<&'static mut [u8; BUFFER_SIZE]>,
        prod_flysky: Producer<'static, [u16; 16], 2>,
        con_flysky: Consumer<'static, [u16; 16], 2>,


        /** Kalman Filter Variables **/
        kalman_timer: timer::CounterMs<TIM4>,
        imu: Lsm6dsox<I2c1>,
        i2c: I2c1,
        x_kalman: KalmanFilter,
        y_kalman: KalmanFilter,

        prod_kalman: Producer<'static, [f32; 2], 5>,
        con_kalman: Consumer<'static, [f32; 2], 5>,

        /** Flight Controller variables **/
        fc_timer: timer::CounterMs<TIM5>,
        
    }


    #[init(local = [
        rx_pool_memory: [u8; 400] = [0; 400],
        queue_kalman: Queue<[f32; 2], 5> = Queue::new(),
        queue_flysky: Queue<[u16; 16], 2> = Queue::new(),
    ])]
    fn init(cx: init::Context) -> (Shared, Local) {

        rtt_init_print!();
        rprintln!("Init");

        let (prod_kalman, con_kalman) = cx.local.queue_kalman.split();
        let (prod_flysky, con_flysky) = cx.local.queue_flysky.split();        

        let token = create_systick_token!();
        Systick::start(cx.core.SYST, 36_000_000, token);

        let dp: hal::pac::Peripherals = cx.device;

        let rcc = dp.RCC.constrain();

        //let clocks = rcc.cfgr.freeze();
        let clocks = rcc.cfgr
            .use_hse(8.MHz())
            .sysclk(36.MHz())
            .pclk1(36.MHz())
            .freeze();


        let gpioa = dp.GPIOA.split();

        // Initialize UART with DMA events
        let rx_pin = gpioa.pa10;
        let mut rx = dp
            .USART1
            .rx(
                rx_pin,
                serial::Config::default()
                    .baudrate(115200.bps())
                    .dma(serial::config::DmaConfig::Rx),
                &clocks,
            )
            .unwrap();

        // Listen UART IDLE event, which will be call USART1 interrupt
        rx.listen_idle();

        let dma2 = StreamsTuple::new(dp.DMA2);

        // Note! It is better to use memory pools, such as heapless::pool::Pool. But it not work with embedded_dma yet.
        // See CHANGELOG of unreleased main branch and issue https://github.com/japaric/heapless/pull/362 for details.
        let rx_buffer1 = cortex_m::singleton!(: [u8; BUFFER_SIZE] = [0; BUFFER_SIZE]).unwrap();
        let rx_buffer2 = cortex_m::singleton!(: [u8; BUFFER_SIZE] = [0; BUFFER_SIZE]).unwrap();

        // Initialize and start DMA stream
        let mut rx_transfer = Transfer::init_peripheral_to_memory(
            dma2.2,
            rx,
            rx_buffer1,
            None,
            DmaConfig::default()
                .memory_increment(true)
                .fifo_enable(true)
                .fifo_error_interrupt(true)
                .transfer_complete_interrupt(true),
        );

        rx_transfer.start(|_rx| {});

        let gpiob = dp.GPIOB.split();

        //// LSM6DSOX IMU \\\\

        let scl = gpiob.pb6.into_open_drain_output();
        let sda = gpiob.pb7.into_open_drain_output();

        let mut i2c = dp.I2C1.i2c(
            (scl, sda),
            Mode::Standard {
                frequency: 200.kHz(),
            },
            &clocks,
        );

        let imu = Lsm6dsox::new(&mut i2c).unwrap();

        let id = imu.read_id(&mut i2c).unwrap();
        rprintln!("id is {:#b}: ", id);

        imu.configure_accel(&mut i2c).unwrap();
        imu.configure_gyro(&mut i2c).unwrap();

        // Kalman Filter
        let x_kalman = KalmanFilter::new();
        let y_kalman = KalmanFilter::new();

        // Kalman Filter Timer
        let mut kalman_timer = dp.TIM4.counter_ms(&clocks);
        kalman_timer.start(2000.millis()).unwrap();
        kalman_timer.listen(Event::Update);

        // Flight Controller Timer

        let mut fc_timer = dp.TIM5.counter_ms(&clocks);
        fc_timer.start(2000.millis()).unwrap();
        fc_timer.listen(Event::Update);

        (
            Shared { rx_transfer },
            Local {
                rx_buffer: Some(rx_buffer2),
                prod_flysky,
                con_flysky,

                imu,
                i2c,
                x_kalman,
                y_kalman,

                prod_kalman,
                con_kalman,
                kalman_timer,

                fc_timer,
            },
        )
    }

    // Important! USART1 and DMA2_STREAM2 should the same interrupt priority!
    #[task(binds = USART1, priority=1, local = [rx_buffer],shared = [rx_transfer])]
    fn usart1(mut cx: usart1::Context) {
        rprintln!("usart1 interrupt");
        
        let transfer = &mut cx.shared.rx_transfer;

        if transfer.is_idle() {
            //rprintln!("transfer is idle");
            // Calc received bytes count
            let bytes_count = BUFFER_SIZE - transfer.number_of_transfers() as usize;

            // Allocate new buffer
            let new_buffer = cx.local.rx_buffer.take().unwrap();

            // Replace buffer and restart DMA stream
            let (buffer, _) = transfer.next_transfer(new_buffer).unwrap();

            //let mut copied_buffer = [0; BUFFER_SIZE];

            //copied_buffer.copy_from_slice(&buffer[..bytes_count]);

            // Get slice for received bytes
            //let _bytes = &buffer[..bytes_count];
            //rprintln!("Data PRE-PROC: {:?}", _bytes);

            //rprintln!("Byte count: {:?}", bytes_count);

            if bytes_count == 32 {
                process_received_bytes::spawn(*buffer).unwrap();
            } else {
                rprintln!("UART Data not 32 Bytes");
            }

            // Free buffer

            *cx.local.rx_buffer = Some(buffer);

        }
    }


    #[task(priority = 1, local=[prod_flysky])]
    async fn process_received_bytes(ctx: process_received_bytes::Context, buffer: [u8; BUFFER_SIZE]) {
        rprintln!("process received bytes");
        //Systick::delay(3.millis()).await;
        let bytes = &buffer[..32];

        //rprintln!("Bytes: {:?}", bytes);

        let mut channel_values: [u16; 16] = [0; 16];

        for i in (0..bytes.len()).step_by(2) {
            if i + 1 < bytes.len() {
                // Extract two bytes from the pair
                let byte1 = bytes[i];
                let byte2 = bytes[i + 1];

                // Combine the bytes by multiplying the second number by 256
                //let combined_value = u16::from(byte1) + u16::from(byte2) * 256;
                let combined_value = u16::from(byte1) | (u16::from(byte2) << 8);

                let channel_index = i / 2;
                //rprintln!("index: {:?}", channel_index);
                // Do something with the combined value (e.g., print or use it)

                channel_values[channel_index] = combined_value;
                //rprintln!("Combined Value: {}", combined_value);
            } else {
                break
            }
        }

        //rprintln!("Channels: {:?}", channel_values);
        match ctx.local.prod_flysky.enqueue(channel_values) {
            Ok(()) => {
                //rprintln!("Data sent");
            }

            Err(_err) => {
                // Other errors occurred, handle them appropriately
                // Example: println!("Error occurred while enqueueing data: {:?}", err);
                rprintln!("Flysky failed to enqueue");
            }
        }
        //let _throttle = map(channel_values[5], 1000, 2000, 0, 100);
        //let _desired_roll = map(channel_values[2], 1000, 2000, 0, 100);
        //let _desired_pitch = map(channel_values[3], 1000, 2000, 0, 100);
        
    }

    /** Mapping Function **/
    /* 
    fn map(x: u16, in_min: u16, in_max: u16, out_min: u16, out_max: u16) -> u16 {
        (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    }
    */

    #[task(binds = DMA2_STREAM2, priority=1, shared = [rx_transfer])]
    fn dma2_stream2(mut cx: dma2_stream2::Context) {
        let transfer = &mut cx.shared.rx_transfer;

        let flags = transfer.flags();
        transfer.clear_flags(DmaFlag::FifoError | DmaFlag::TransferComplete);
        if flags.is_transfer_complete() {
            // Buffer is full, but no IDLE received!
            // You can process this data or discard data (ignore transfer complete interrupt and wait IDLE).

            // Note! If you want process this data, it is recommended to use double buffering.
            // See Transfer::init_peripheral_to_memory for details.
        }
    }
    

    #[task(binds = TIM4, local=[kalman_timer, imu, i2c, x_kalman, y_kalman, prod_kalman], priority = 1)]
    fn sensor_fusion(ctx: sensor_fusion::Context) {
        ctx.local.kalman_timer.clear_all_flags();
        ctx.local.kalman_timer.start(100.millis()).unwrap();

        rprintln!("Sensor fusion");

        let delta_sec = 0.1;

        let accel_data: [f32; 3];
        let gyro_data: [f32; 3];

        let x_accel: f32;
        let y_accel: f32;

        let imu = ctx.local.imu;

        accel_data = imu.read_accel(ctx.local.i2c).unwrap();
        gyro_data = imu.read_gyro(ctx.local.i2c).unwrap();

        y_accel = atanf(accel_data[0] / sqrtf(accel_data[1] * accel_data[1] + accel_data[2] * accel_data[2])) * 180.0 / PI;
        x_accel = atanf(accel_data[1] / sqrtf(accel_data[0] * accel_data[0] + accel_data[2] * accel_data[2])) * 180.0/ PI;

        ctx.local.x_kalman.process_posterior_state(gyro_data[0], x_accel, delta_sec);
        ctx.local.y_kalman.process_posterior_state(gyro_data[1], y_accel, delta_sec);

        match ctx.local.prod_kalman.enqueue([ctx.local.x_kalman.get_angle(), ctx.local.y_kalman.get_angle()]) {
            Ok(()) => {
                //rprintln!("Data sent");
            }

            Err(_err) => {
                // Other errors occurred, handle them appropriately
                // Example: println!("Error occurred while enqueueing data: {:?}", err);
                rprintln!("IMU Data failed to send");
            }
        }
    }

    #[task(binds = TIM5, local = [fc_timer, con_kalman, con_flysky], priority = 1)]
    fn flight_controller(ctx: flight_controller::Context) {
        ctx.local.fc_timer.clear_all_flags();
        ctx.local.fc_timer.start(1000.millis()).unwrap();

        rprintln!("FC");

        if let Some(data) = ctx.local.con_kalman.dequeue() {
            rprintln!("Kalman Data: {:?}", data);
        }
        
        if let Some(data) = ctx.local.con_flysky.dequeue() {
            rprintln!("Flysky Data: {:?}", data);
        }
        
        // Print Desired Angle (Kalman Filter output)

        // Print Actual Angle (Remote Controller post-processing)

    }
}
