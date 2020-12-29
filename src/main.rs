#![no_main]
#![no_std]

// set the panic handler
use panic_semihosting as _;

use cortex_m_rt::entry;
use embedded_hal::digital::v2::*;
use nb::block;
use stm32f1xx_hal::{pac, prelude::*};

#[entry]
fn main() -> ! {
    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();
    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();

    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    let mut gpioa = dp.GPIOA.split(&mut rcc.apb2);
    let mut gpioc = dp.GPIOC.split(&mut rcc.apb2);

    // Configure pc13 as output via CR high register.
    let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);

    let mut pa4 = gpioa.pa4.into_push_pull_output(&mut gpioa.crl);
    let pa10 = gpioa.pa10.into_pull_up_input(&mut gpioa.crh);

    let mut timer = stm32f1xx_hal::timer::Timer::syst(cp.SYST, &clocks).start_count_down(90.hz());

    led.set_high().unwrap(); // LED off
    pa4.set_low().unwrap(); // Oscill low

    const MAX_STEPS: i32 = 10;
    let mut counter = 0;
    let mut steps = 1;

    loop {
        block!(timer.wait()).unwrap();
        counter += 1;
        if counter >= steps {
            counter = 0;
            led.toggle().unwrap();
            pa4.toggle().unwrap();
        }
        if pa10.is_low().unwrap() {
            steps = (steps + 1) % (MAX_STEPS + 1);
        }
    }
}
