#![deny(unsafe_code)]
#![deny(warnings)]
#![no_main]
#![no_std]

use panic_halt as _;

#[rtic::app(device = stm32f4xx_hal::pac, peripherals = true, dispatchers = [TIM2])]
mod app {

    use hal::{
        dma::{config::DmaConfig, DmaFlag, PeripheralToMemory, Stream2, StreamsTuple, Transfer},
        pac::{DMA2, USART1},
        prelude::*,
        rcc::RccExt,
        serial,
    };
    use rtt_target::{rprintln, rtt_init_print};
    use stm32f4xx_hal as hal;

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
        rx_buffer: Option<&'static mut [u8; BUFFER_SIZE]>,
    }

    #[init(local = [
        rx_pool_memory: [u8; 400] = [0; 400],
    ])]
    fn init(cx: init::Context) -> (Shared, Local) {
        rtt_init_print!();
        rprintln!("Init");
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

        (
            Shared { rx_transfer },
            Local {
                rx_buffer: Some(rx_buffer2),
            },
        )
    }

    // Important! USART1 and DMA2_STREAM2 should the same interrupt priority!
    #[task(binds = USART1, priority=1, local = [rx_buffer],shared = [rx_transfer])]
    fn usart1(mut cx: usart1::Context) {
        rprintln!("usart1 interrupt");
        let transfer = &mut cx.shared.rx_transfer;

        if transfer.is_idle() {
            rprintln!("idle");
            // Calc received bytes count
            let bytes_count = BUFFER_SIZE - transfer.number_of_transfers() as usize;

            // Allocate new buffer
            let new_buffer = cx.local.rx_buffer.take().unwrap();

            // Replace buffer and restart DMA stream
            let (buffer, _) = transfer.next_transfer(new_buffer).unwrap();

            // Get slice for received bytes
            let bytes = &buffer[..bytes_count];
            //let trimmed_bytes = &bytes[2..bytes.len() - 2];

            let mut channel_values: [u16; 16] = [0; 16];

            for i in (0..bytes.len()).step_by(2) {
                // Extract two bytes from the pair
                let byte1 = bytes[i];
                let byte2 = bytes[i + 1];
    
                // Combine the bytes by multiplying the second number by 256
                let combined_value = u16::from(byte1) + u16::from(byte2) * 256;

                let channel_index = i / 2;
                //rprintln!("index: {:?}", channel_index);
                // Do something with the combined value (e.g., print or use it)

                channel_values[channel_index] = combined_value;
                //rprintln!("Combined Value: {}", combined_value);
            }
            //rprintln!("Bytes: {:?}", bytes);
            rprintln!("Channels: {:?}", channel_values);
        
            
            
            // Do something with received bytes
            // For example, parse it or send (buffer, bytes_count) to lock-free queue.
            /*for &byte in bytes {
                rprintln!("{:?} ", byte);
            }*/

            // Free buffer
            *cx.local.rx_buffer = Some(buffer);

            
        }
    }
    /* 
    fn process_received_bytes(bytes: &[u8]) {
        rprintln!("process received bytes");
        // Process the bytes in pairs
        for i in (0..bytes.len()).step_by(2) {
            // Extract two bytes from the pair
            let byte1 = bytes[i];
            let byte2 = bytes[i + 1];

            // Combine the bytes by multiplying the second number by 256
            let combined_value = u16::from(byte1) + u16::from(byte2) * 256;

            // Do something with the combined value (e.g., print or use it)
            rprintln!("Combined Value: {}", combined_value);
        }
    }
    */

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
}