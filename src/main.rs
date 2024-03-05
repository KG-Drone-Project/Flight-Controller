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
    use stm32f4xx_hal as hal;

    use hal::{
        dma::{config::DmaConfig, DmaFlag, PeripheralToMemory, Stream2, StreamsTuple, Transfer}, i2c::{I2c1, Mode}, pac::{DMA2, TIM2, USART1}, prelude::*, rcc::RccExt, serial, timer::{self, Event}
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


        /** Kalman Filter Variables **/
        timer: timer::CounterMs<TIM2>,
        imu: Lsm6dsox<I2c1>,
        i2c: I2c1,
        x_kalman: KalmanFilter,
        y_kalman: KalmanFilter,

        prod_kalman: Producer<'static, [f32; 2], 5>,
        con_kalman: Consumer<'static, [f32; 2], 5>
    }


    #[init(local = [
        rx_pool_memory: [u8; 400] = [0; 400],
        queue_kalman: Queue<[f32; 2], 5> = Queue::new()
    ])]
    fn init(cx: init::Context) -> (Shared, Local) {

        rtt_init_print!();
        rprintln!("Init");

        let (prod_kalman, con_kalman) = cx.local.queue_kalman.split();

        let dp: hal::pac::Peripherals = cx.device;

        let rcc = dp.RCC.constrain();
        let clocks = rcc.cfgr.freeze();


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

        //// LSM6DSOX IMU

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
        let mut timer = dp.TIM2.counter_ms(&clocks);
        timer.start(2000.millis()).unwrap();
        timer.listen(Event::Update);

        (
            Shared { rx_transfer },
            Local {
                rx_buffer: Some(rx_buffer2),
                imu,
                i2c,
                x_kalman,
                y_kalman,

                prod_kalman,
                con_kalman,
                timer
            },
        )
    }

    #[idle(local = [con_kalman])]
    fn idle(ctx: idle::Context) -> ! {


        rprintln!("idle");

        loop {

            //rprintln!("Kalman Filter x: {:?}, y: {:?}", *x_kal, *y_kal);
            if let Some(data) = ctx.local.con_kalman.dequeue() {
                rprintln!("Data: {:?}", data);
            }

        }
    }

    // Important! USART1 and DMA2_STREAM2 should the same interrupt priority!
    #[task(binds = USART1, priority=1, local = [rx_buffer],shared = [rx_transfer])]
    fn usart1(mut cx: usart1::Context) {
        rprintln!("usart1 interrupt");
        let transfer = &mut cx.shared.rx_transfer;

        if transfer.is_idle() {
            rprintln!("tansfer is idle");
            // Calc received bytes count
            let bytes_count = BUFFER_SIZE - transfer.number_of_transfers() as usize;

            // Allocate new buffer
            let new_buffer = cx.local.rx_buffer.take().unwrap();

            // Replace buffer and restart DMA stream
            let (buffer, _) = transfer.next_transfer(new_buffer).unwrap();

            //let mut copied_buffer = [0; BUFFER_SIZE];

            //copied_buffer.copy_from_slice(&buffer[..bytes_count]);

            // Get slice for received bytes
            let _bytes = &buffer[..bytes_count];
            rprintln!("Data PRE-PROC: {:?}", _bytes);

            rprintln!("Byte count: {:?}", bytes_count);

            if bytes_count == 32 {
                process_received_bytes::spawn(*buffer).unwrap();
            }

            // Free buffer

            *cx.local.rx_buffer = Some(buffer);

        }
    }


    #[task(priority = 2)]
    async fn process_received_bytes(mut _ctx: process_received_bytes::Context, buffer: [u8; BUFFER_SIZE]) {
        rprintln!("process received bytes");

        let bytes = &buffer[..32];

        rprintln!("Bytes: {:?}", bytes);

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

        rprintln!("Channels: {:?}", channel_values);
    }

    #[task(binds = DMA2_STREAM2, priority=1,shared = [rx_transfer])]
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


    #[task(binds = TIM2, local=[timer, imu, i2c, x_kalman, y_kalman, prod_kalman], priority = 1)]
    fn timer_expired(ctx: timer_expired::Context) {
        ctx.local.timer.clear_all_flags();
        ctx.local.timer.start(12.millis()).unwrap();

        let delta_sec = 0.012;

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

            Err(err) => {
                // Other errors occurred, handle them appropriately
                // Example: println!("Error occurred while enqueueing data: {:?}", err);
                rprintln!("IMU Data failed to send: {:?}", err);
            }
        }
    }
}
