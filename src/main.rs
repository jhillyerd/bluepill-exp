#![no_main]
#![no_std]

// set the panic handler
use panic_semihosting as _;

use core::mem::MaybeUninit;
use core::sync::atomic::{AtomicUsize, Ordering};
use cortex_m_rt::entry;
use embedded_hal::digital::v2::*;
use nb::block;
use pac::interrupt;
use stm32f1xx_hal::{gpio::*, pac, prelude::*};

const MAX_STEPS: usize = 10;
static STEPS: AtomicUsize = AtomicUsize::new(1);

static mut BUTTON: MaybeUninit<gpioa::PA10<Input<PullUp>>> = MaybeUninit::uninit();

#[entry]
fn main() -> ! {
    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();
    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();

    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    let mut afio = dp.AFIO.constrain(&mut rcc.apb2);
    let mut gpioa = dp.GPIOA.split(&mut rcc.apb2);
    let mut gpioc = dp.GPIOC.split(&mut rcc.apb2);

    // Configure pc13 as output via CR high register.
    let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);

    let mut pa4 = gpioa.pa4.into_push_pull_output(&mut gpioa.crl);

    let mut timer = stm32f1xx_hal::timer::Timer::syst(cp.SYST, &clocks).start_count_down(90.hz());

    led.set_high().unwrap(); // LED off
    pa4.set_low().unwrap(); // Oscill low

    // Setup pa10 button interrupt.  Safety: Will not be accessed from anywhere but the interrupt
    // handler after we initialize.
    {
        let button = unsafe { &mut *BUTTON.as_mut_ptr() };
        *button = gpioa.pa10.into_pull_up_input(&mut gpioa.crh);
        button.make_interrupt_source(&mut afio);
        button.trigger_on_edge(&dp.EXTI, Edge::FALLING);
        button.enable_interrupt(&dp.EXTI);
    }

    unsafe {
        // Pins 10-15 are on the 15-10 EXTI.
        pac::NVIC::unmask(pac::Interrupt::EXTI15_10);
    }

    let mut counter = 0;

    loop {
        block!(timer.wait()).unwrap();
        counter += 1;
        if counter >= STEPS.load(Ordering::Relaxed) {
            counter = 0;
            led.toggle().unwrap();
            pa4.toggle().unwrap();
        }
    }
}

#[interrupt]
fn EXTI15_10() {
    let button = unsafe { &mut *BUTTON.as_mut_ptr() };

    if button.check_interrupt() {
        STEPS
            .fetch_update(Ordering::Relaxed, Ordering::Relaxed, |steps| {
                Some((steps % MAX_STEPS) + 1)
            })
            .unwrap();
        button.clear_interrupt_pending_bit();
    }
}
