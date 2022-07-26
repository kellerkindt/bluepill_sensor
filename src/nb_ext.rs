#[macro_export]
macro_rules! block_while {
    ($c:expr, $e:expr) => {
        loop {
            #[allow(unreachable_patterns)]
            match $e {
                Err(nb::Error::Other(e)) =>
                {
                    #[allow(unreachable_code)]
                    break Err(nb::Error::Other(e))
                }
                Err(nb::Error::WouldBlock) => {
                    if !$c {
                        break Err(nb::Error::WouldBlock);
                    }
                }
                Ok(x) => break Ok(x),
            }
        }
    };
}
