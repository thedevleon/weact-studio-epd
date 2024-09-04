#![no_std]
#![no_main]

use core::fmt::Write;
use display_interface_spi::SPIInterface;
use embassy_executor::Spawner;
use embassy_time::{Delay, Duration, Timer};
use embedded_graphics::{
    geometry::Point,
    mono_font::MonoTextStyle,
    text::{Text, TextStyle},
    Drawable,
};
use embedded_hal_bus::spi::ExclusiveDevice;
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    dma::*,
    dma_buffers,
    gpio::{Input, Io, Level, Output, NO_PIN},
    peripherals::Peripherals,
    prelude::*,
    spi::{master::Spi, SpiMode},
    system::SystemControl,
    timer::timg::TimerGroup,
};
use heapless::String;
use profont::PROFONT_24_POINT;
use weact_studio_epd::{graphics::Display290BlackWhite, Color};
use weact_studio_epd::{graphics::DisplayRotation, WeActStudio290BlackWhiteDriver};

#[esp_hal_embassy::main]
async fn main(_spawner: Spawner) {
    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);
    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
    let clocks = ClockControl::max(system.clock_control).freeze();
    let dma = Dma::new(peripherals.DMA);
    let timg0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    esp_hal_embassy::init(&clocks, timg0.timer0);

    esp_println::logger::init_logger_from_env();

    log::info!("Intializing SPI Bus...");

    let sclk = io.pins.gpio6;
    let mosi = io.pins.gpio7;
    let cs = io.pins.gpio15;
    let dc = io.pins.gpio21;
    let rst = io.pins.gpio22;
    let busy = io.pins.gpio23;

    // Convert pins into InputPins and OutputPins
    /*
        CS: OutputPin,
        BUSY: InputPin,
        DC: OutputPin,
        RST: OutputPin,
    */
    let cs = Output::new(cs, Level::High);
    let busy = Input::new(busy, esp_hal::gpio::Pull::Up);
    let dc = Output::new(dc, Level::Low);
    let rst = Output::new(rst, Level::High);

    let (tx_buffer, tx_descriptors, rx_buffer, rx_descriptors) = dma_buffers!(32000);
    let dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();
    let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();

    log::info!("Intializing SPI...");
    let spi_bus = Spi::new(peripherals.SPI2, 100.kHz(), SpiMode::Mode0, &clocks)
        .with_pins(
            Some(sclk),
            Some(mosi),
            NO_PIN,
            NO_PIN, // cs is handled by the exclusive device
        )
        .with_dma(
            dma.channel0
                .configure_for_async(false, DmaPriority::Priority0),
        )
        .with_buffers(dma_tx_buf, dma_rx_buf);

    let spi_device = ExclusiveDevice::new(spi_bus, cs, Delay).expect("SPI device initialize error");
    let spi_interface = SPIInterface::new(spi_device, dc);

    // Setup EPD
    log::info!("Intializing EPD...");
    let mut driver = WeActStudio290BlackWhiteDriver::new(spi_interface, busy, rst, Delay);
    let mut display = Display290BlackWhite::new();
    display.set_rotation(DisplayRotation::Rotate90);
    driver.init().await.unwrap();

    let style = MonoTextStyle::new(&PROFONT_24_POINT, Color::Black);
    let _ = Text::with_text_style(
        "Hello World!",
        Point::new(8, 68),
        style,
        TextStyle::default(),
    )
    .draw(&mut display);

    driver.full_update(&display).await.unwrap();

    log::info!("Sleeping for 5s...");
    driver.sleep().await.unwrap();
    Timer::after(Duration::from_millis(5_000)).await;

    let mut n: u8 = 0;
    loop {
        log::info!("Wake up!");
        driver.wake_up().await.unwrap();

        display.clear(Color::White);

        let mut string_buf = String::<30>::new();
        write!(string_buf, "Hello World {}!", n).unwrap();
        let _ = Text::with_text_style(&string_buf, Point::new(8, 68), style, TextStyle::default())
            .draw(&mut display)
            .unwrap();
        string_buf.clear();

        driver.full_update(&display).await.unwrap();

        n = n.wrapping_add(1); // Wrap from 0..255

        log::info!("Sleeping for 5s...");
        driver.sleep().await.unwrap();
        Timer::after(Duration::from_millis(5_000)).await;
    }
}
