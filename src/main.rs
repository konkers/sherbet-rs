#![no_std]
#![no_main]

use core::cmp::{max, min};
use core::sync::atomic::{AtomicBool, Ordering};

use defmt::{info, unwrap, warn};
use embassy_executor::Spawner;
use embassy_futures::join::{join, join3, join4};
use embassy_rp::adc::{self, Adc};
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Input, Level, Output, Pin, Pull};
use embassy_rp::peripherals::USB;
use embassy_rp::usb::{self, Driver};
use embassy_time::{Duration, Instant, Timer};
use embassy_usb::class::hid::{HidReaderWriter, HidWriter, ReportId, RequestHandler, State};
use embassy_usb::control::OutResponse;
use embassy_usb::{Builder, Config, Handler};
use embedded_hal::digital::{InputPin, OutputPin};
use keyberon::debounce::Debouncer;
use keyberon::key_code::KbHidReport;
use keyberon::keyboard::Keyboard;
use keyberon::layout::{Event, Layout};
use keyberon::matrix::{AsyncDelay, Matrix};
use packed_struct::PackedStruct;
use usbd_hid::descriptor::{KeyboardReport, SerializedDescriptor};
use usbd_human_interface_device::device::joystick::{JoystickReport, JOYSTICK_DESCRIPTOR};

use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => usb::InterruptHandler<USB>;
    ADC_IRQ_FIFO => adc::InterruptHandler;
});

pub(crate) trait StaticArray {
    const LEN: usize;
}

impl<T, const N: usize> StaticArray for [T; N] {
    const LEN: usize = N;
}

pub enum CustomActions {
    Reset,
}

#[rustfmt::skip]
pub static LAYERS: keyberon::layout::Layers<7, 4, 1, CustomActions> = keyberon::layout::layout! {
    { // 0
        [1   2   3   4    5      6       No]
        [7   8   9   0    Minus  Bslash  No]
        [F1  F2  F3  F4   F5     F6      No]
        [F7  F8  F9  F10  F11    F12     Kp0]
    }
};

struct ScanDelay {}

impl AsyncDelay for ScanDelay {
    async fn delay(&self) {
        Timer::after_micros(5).await;
    }
}

async fn keyboard_task<
    'd,
    E: defmt::Format,
    C: InputPin<Error = E>,
    R: OutputPin<Error = E>,
    D: embassy_usb_driver::Driver<'d>,
    const CS: usize,
    const RS: usize,
    const WRITE_N: usize,
>(
    mut matrix: Matrix<C, R, CS, RS>,
    mut js_button_pin: impl InputPin<Error = E>,
    mut hid_writer: HidWriter<'d, D, WRITE_N>,
) {
    let mut layout = Layout::new(&LAYERS);
    let mut debouncer = Debouncer::new([[false; CS]; RS], [[false; CS]; RS], 4);
    let mut keyboard: Keyboard<()> = Keyboard::new(());
    let mut prev_js_button_state = false;

    const SCAN_PERIOD: Duration = Duration::from_millis(1);
    let mut next_scan = Instant::now();
    loop {
        let keys_pressed = unwrap!(matrix.get_with_async_delay(&ScanDelay {}).await);
        let events = debouncer.events(keys_pressed);

        let events = events.map(|event| match event {
            Event::Press(i, j) => Event::Press(j, i),
            Event::Release(i, j) => Event::Release(j, i),
        });

        for event in events {
            layout.event(event);
        }

        if let Ok(js_button_state) = js_button_pin.is_low() {
            if js_button_state != prev_js_button_state {
                match js_button_state {
                    true => layout.event(Event::Press(3, 6)),
                    false => layout.event(Event::Release(3, 6)),
                }
                prev_js_button_state = js_button_state;
            }
        }

        // TODO: handle custom events.
        let _ = layout.tick();

        let report: KbHidReport = layout.keycodes().collect();
        if keyboard.set_keyboard_report(report.clone()) {
            if let Err(e) = hid_writer.write(report.as_bytes()).await {
                warn!("Error writing keyboard HID report:{}", e);
            }
        }

        // Skip scans that we may have missed due to USB events piling up.
        while next_scan < Instant::now() {
            next_scan += SCAN_PERIOD;
        }

        Timer::at(next_scan).await;
    }
}

fn scale_axis(val: u16, invert: bool, zero_val: u16, dead_band: u16, range: u16) -> i8 {
    let mut val = val as i32;
    val -= zero_val as i32;
    if val.abs() < dead_band as i32 {
        return 0;
    }

    val *= 128;
    val /= range as i32;

    let val = match invert {
        true => -val,
        false => val,
    };

    val.clamp(i8::MIN as i32, i8::MAX as i32) as i8
}

async fn joystick_task<'d, D: embassy_usb_driver::Driver<'d>, const WRITE_N: usize>(
    mut adc: Adc<'_, adc::Async>,
    mut x_channel: adc::Channel<'_>,
    mut y_channel: adc::Channel<'_>,
    mut hid_writer: HidWriter<'d, D, WRITE_N>,
) {
    const SCAN_PERIOD: Duration = Duration::from_millis(10);
    let mut next_scan = Instant::now();
    // let mut min_x = u16::MAX;
    // let mut min_y = u16::MAX;
    // let mut max_x = u16::MIN;
    // let mut max_y = u16::MIN;
    let mut last_report = JoystickReport {
        x: 0,
        y: 0,
        buttons: 0,
    };
    loop {
        // Skip scans that we may have missed due to USB events piling up.
        while next_scan < Instant::now() {
            next_scan += SCAN_PERIOD;
        }

        Timer::at(next_scan).await;

        let Ok(x_val) = adc.read(&mut x_channel).await else {
            warn!("Error reading x channel");
            continue;
        };
        let Ok(y_val) = adc.read(&mut y_channel).await else {
            warn!("Error reading y channel");
            continue;
        };
        // min_x = min(min_x, x_val);
        // min_y = min(min_y, y_val);
        // max_x = max(max_x, x_val);
        // max_y = max(max_y, y_val);
        // info!(
        //     "adc {}({}-{}), {}({}-{})",
        //     x_val, min_x, max_x, y_val, min_y, max_y
        // );
        let x = scale_axis(x_val, true, 1948, 32, 1200);
        let y = scale_axis(y_val, true, 1942, 32, 1200);
        let report = JoystickReport { x, y, buttons: 0 };
        if report != last_report {
            let Ok(data) = report.pack() else {
                warn!("Can't pack report");
                continue;
            };

            if let Err(e) = hid_writer.write(&data).await {
                warn!("Error writing joystick HID report:{}", e);
                continue;
            }
            last_report = report;
        }
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    // Create the driver, from the HAL.
    let driver = Driver::new(p.USB, Irqs);

    // Create embassy-usb Config
    let mut config = Config::new(0xc0de, 0xcafe);
    config.manufacturer = Some("Konkers");
    config.product = Some("Sherbet Gaming Keypad");
    config.serial_number = Some("12345678");
    config.max_power = 100;
    config.max_packet_size_0 = 64;

    // Create embassy-usb DeviceBuilder using the driver and config.
    // It needs some buffers for building the descriptors.
    let mut config_descriptor = [0; 256];
    let mut bos_descriptor = [0; 256];
    // You can also add a Microsoft OS descriptor.
    let mut msos_descriptor = [0; 256];
    let mut control_buf = [0; 64];
    let mut request_handler = MyRequestHandler {};
    let mut device_handler = MyDeviceHandler::new();

    // State objects need to oulive the builder so we declare them first.
    let mut kb_state = State::new();
    let mut js_state = State::new();

    let mut builder = Builder::new(
        driver,
        config,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut msos_descriptor,
        &mut control_buf,
    );

    builder.handler(&mut device_handler);

    let kb_config = embassy_usb::class::hid::Config {
        report_descriptor: KeyboardReport::desc(),
        request_handler: None,
        poll_ms: 1,
        max_packet_size: 64,
    };
    let kb_hid = HidReaderWriter::<_, 1, 8>::new(&mut builder, &mut kb_state, kb_config);

    let js_config = embassy_usb::class::hid::Config {
        report_descriptor: JOYSTICK_DESCRIPTOR,
        request_handler: None,
        poll_ms: 10,
        max_packet_size: 64,
    };
    let js_writer = HidWriter::<
        _,
        { <<JoystickReport as PackedStruct>::ByteArray as StaticArray>::LEN },
    >::new(&mut builder, &mut js_state, js_config);

    // Build the builder.
    let mut usb = builder.build();

    // Run the USB device.
    let usb_fut = usb.run();

    // Set up the signal pin that will be used to trigger the keyboard.
    let mut signal_pin = Input::new(p.PIN_16, Pull::None);

    // Enable the schmitt trigger to slightly debounce.
    signal_pin.set_schmitt(true);

    let (kb_reader, kb_writer) = kb_hid.split();

    let matrix = Matrix::new(
        [
            Input::new(p.PIN_8.degrade(), Pull::Up),
            Input::new(p.PIN_9.degrade(), Pull::Up),
            Input::new(p.PIN_10.degrade(), Pull::Up),
            Input::new(p.PIN_11.degrade(), Pull::Up),
        ],
        [
            Output::new(p.PIN_2.degrade(), Level::High),
            Output::new(p.PIN_3.degrade(), Level::High),
            Output::new(p.PIN_4.degrade(), Level::High),
            Output::new(p.PIN_5.degrade(), Level::High),
            Output::new(p.PIN_6.degrade(), Level::High),
            Output::new(p.PIN_7.degrade(), Level::High),
        ],
    )
    .unwrap();

    let kb_in_fut = keyboard_task(matrix, Input::new(p.PIN_12.degrade(), Pull::Up), kb_writer);
    let kb_out_fut = async {
        kb_reader.run(false, &mut request_handler).await;
    };

    let adc = Adc::new(p.ADC, Irqs, adc::Config::default());
    let p26 = adc::Channel::new_pin(p.PIN_26, Pull::None);
    let p27 = adc::Channel::new_pin(p.PIN_27, Pull::None);
    let joystick_fut = joystick_task(adc, p27, p26, js_writer);

    // Run everything concurrently.
    join4(usb_fut, joystick_fut, kb_in_fut, kb_out_fut).await;
}

struct MyRequestHandler {}

impl RequestHandler for MyRequestHandler {
    fn get_report(&mut self, id: ReportId, _buf: &mut [u8]) -> Option<usize> {
        info!("Get report for {:?}", id);
        None
    }

    fn set_report(&mut self, id: ReportId, data: &[u8]) -> OutResponse {
        info!("Set report for {:?}: {=[u8]}", id, data);
        OutResponse::Accepted
    }

    fn set_idle_ms(&mut self, id: Option<ReportId>, dur: u32) {
        info!("Set idle rate for {:?} to {:?}", id, dur);
    }

    fn get_idle_ms(&mut self, id: Option<ReportId>) -> Option<u32> {
        info!("Get idle rate for {:?}", id);
        None
    }
}

struct MyDeviceHandler {
    configured: AtomicBool,
}

impl MyDeviceHandler {
    fn new() -> Self {
        MyDeviceHandler {
            configured: AtomicBool::new(false),
        }
    }
}

impl Handler for MyDeviceHandler {
    fn enabled(&mut self, enabled: bool) {
        self.configured.store(false, Ordering::Relaxed);
        if enabled {
            info!("Device enabled");
        } else {
            info!("Device disabled");
        }
    }

    fn reset(&mut self) {
        self.configured.store(false, Ordering::Relaxed);
        info!("Bus reset, the Vbus current limit is 100mA");
    }

    fn addressed(&mut self, addr: u8) {
        self.configured.store(false, Ordering::Relaxed);
        info!("USB address set to: {}", addr);
    }

    fn configured(&mut self, configured: bool) {
        self.configured.store(configured, Ordering::Relaxed);
        if configured {
            info!(
                "Device configured, it may now draw up to the configured current limit from Vbus."
            )
        } else {
            info!("Device is no longer configured, the Vbus current limit is 100mA.");
        }
    }
}
