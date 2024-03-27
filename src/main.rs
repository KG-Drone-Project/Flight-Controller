#![deny(unsafe_code)]
#![deny(warnings)]
#![allow(unused_assignments)]
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
        i2c::{I2c1, Mode}, 
        pac::{TIM1, TIM2, TIM4, USART1}, 
        prelude::*, rcc::RccExt, 
        serial::{Config, Rx, Serial}, 
        timer::{self, Channel1, Channel2, Event, PwmChannel}
    };

    use rtt_target::{rprintln, rtt_init_print};

    /** Kalman Filter Libraries **/
    use libm::{atanf, sqrtf};
    use crate::kalman::KalmanFilter;
    use lsm6dsox_driver::Lsm6dsox;

    #[shared]
    struct Shared {
    }

    #[local]
    struct Local {
        /** FlySky Buffer **/
        rx: Rx<USART1, u8>,
        prod_flysky: Producer<'static, [f32; 3], 2>,
        con_flysky: Consumer<'static, [f32; 3], 2>,


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

        /** ESC **/
        m1: PwmChannel<TIM1, 0>,
        m2: PwmChannel<TIM1, 1>,
        m3: PwmChannel<TIM2, 0>,
        m4: PwmChannel<TIM2, 1>

    }


    #[init(local = [
        rx_pool_memory: [u8; 400] = [0; 400],
        queue_kalman: Queue<[f32; 2], 5> = Queue::new(),
        queue_flysky: Queue<[f32; 3], 2> = Queue::new(),
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
        let rx_pin = gpioa.pa10; // RX

        // Configure USART1 with a baud rate of 115200
        let rx: Rx<USART1, u8> =
            Serial::rx(dp.USART1, rx_pin, Config::default().baudrate(115200.bps()), &clocks).unwrap();


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
        kalman_timer.start(10000.millis()).unwrap();
        kalman_timer.listen(Event::Update);

        // Motor Timer Setup
        let channels = (Channel1::new(gpioa.pa8), Channel2::new(gpioa.pa9));
        let channels_1 = (Channel1::new(gpioa.pa0), Channel2::new(gpioa.pa1));
        
        let pwm = dp.TIM1.pwm_hz(channels, 50.Hz(), &clocks).split();
        let pwm_1 = dp.TIM2.pwm_hz(channels_1, 50.Hz(), &clocks).split();

        let (mut m1, mut m2) = pwm;
        let (mut m3, mut m4) = pwm_1;

        m1.enable();
        m2.enable();
        m3.enable();
        m4.enable();

        rprintln!("Zero signal");
        m1.set_duty(m1.get_max_duty() / 20);
        m2.set_duty(m2.get_max_duty() / 20);
        m3.set_duty(m3.get_max_duty() / 20);
        m4.set_duty(m4.get_max_duty() / 20);

        //rprintln!("m1 Value: {:?}", [m1.get_max_duty() / 20, m1.get_max_duty() / 10]);

        // Flight Controller Timer

        let mut fc_timer = dp.TIM5.counter_ms(&clocks);
        fc_timer.start(10000.millis()).unwrap();
        fc_timer.listen(Event::Update);

        process_received_bytes::spawn().ok();

        (
            Shared {  },
            Local {
                rx,
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
                
                m1, m2, m3, m4
            },
        )
    }

    #[task(priority = 1, local=[prod_flysky, rx])]
    async fn process_received_bytes(ctx: process_received_bytes::Context) {

        let rx = ctx.local.rx;


        loop {
            rprintln!("process received bytes");
            

            let mut bytes = [0u8; 32];
            let mut index = 0;


            while index != 32 {
                if let Ok(byte) = rx.read() {
                    if index == 0 && byte != 32 {
                        // Ignore bytes until the first byte is 32 (space character)
                        continue;
                    }
        
                    bytes[index] = byte;
                    index += 1;
        
                    // If the 32nd byte is encountered, process the buffer
                    if index == 32 {
                        //rprintln!("Data: {:?}", bytes);
                        //index = 0; // Reset index for the next packet
                    }
                }
            
            }

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

            let _throttle = channel_values[3] as f32;
            let _desired_roll = map(channel_values[1] as f32, 1000., 2000., -15., 15.);
            let _desired_pitch = map(channel_values[2] as f32, 1000., 2000., -15., 15.);

            match ctx.local.prod_flysky.enqueue([_throttle, _desired_roll, _desired_pitch]) {
                Ok(()) => {
                    //rprintln!("Data sent");
                }

                Err(_err) => {
                    // Other errors occurred, handle them appropriately
                    // Example: println!("Error occurred while enqueueing data: {:?}", err);
                    rprintln!("Flysky failed to enqueue");
                }
            }

            Systick::delay(15.millis()).await;
        }
        
    }

    /** Mapping Function **/
    
    fn map(x: f32, in_min: f32, in_max: f32, out_min: f32, out_max: f32) -> f32 {
        (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    }
    


    #[task(binds = TIM4, local=[kalman_timer, imu, i2c, x_kalman, y_kalman, prod_kalman], priority = 1)]
    fn sensor_fusion(ctx: sensor_fusion::Context) {
        ctx.local.kalman_timer.clear_all_flags();
        ctx.local.kalman_timer.start(15.millis()).unwrap();

        rprintln!("Sensor fusion");

        let delta_sec = 0.15;

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

    #[task(binds = TIM5, local = [fc_timer, con_kalman, con_flysky, m1, m2, m3, m4], priority = 1)]
    fn flight_controller(ctx: flight_controller::Context) {
        ctx.local.fc_timer.clear_all_flags();
        ctx.local.fc_timer.start(15.millis()).unwrap();

        let m1 = ctx.local.m1;
        let m2 = ctx.local.m2;
        let m3 = ctx.local.m3;
        let m4 = ctx.local.m4;

        let mut kalman_data: [f32; 2] = [0., 0.];
        let mut fly_sky_data: [f32; 3] = [0., 0., 0.];

        rprintln!("FC");

        if let Some(data) = ctx.local.con_kalman.dequeue() {
            kalman_data = data;
            rprintln!("Kalman Data: {:?}", data);
        }
        
        if let Some(data) = ctx.local.con_flysky.dequeue() {
            fly_sky_data = data;
            rprintln!("Flysky Data: {:?}", data);
            
        }
        //let _esc_value = map(fly_sky_data[0], 1000., 2000., m1.get_max_duty() as f32 / 20., m1.get_max_duty() as f32 / 10.) as u16;
        m1.set_duty(map(fly_sky_data[0], 1000., 2000., m1.get_max_duty() as f32 / 20., m1.get_max_duty() as f32 / 10.) as u16); 
        m2.set_duty(map(fly_sky_data[0], 1000., 2000., m2.get_max_duty() as f32 / 20., m2.get_max_duty() as f32 / 10.) as u16);
        m3.set_duty(map(fly_sky_data[0], 1000., 2000., m3.get_max_duty() as f32 / 20., m3.get_max_duty() as f32 / 10.) as u16);
        m4.set_duty(map(fly_sky_data[0], 1000., 2000., m4.get_max_duty() as f32 / 20., m4.get_max_duty() as f32 / 10.) as u16);


        // Get Error
        let _error_roll = fly_sky_data[1] - kalman_data[0];
        let _error_pitch = fly_sky_data[2] - kalman_data[1];

        // PID


    }
}
