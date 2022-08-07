#![no_std]

macro_rules! getter_setter {
    ($getter:ident, $setter:ident, $with:ident, $offset:ident = $offset_value:expr, $len:ident = 1) => {
        const $offset: usize = $offset_value;
        const $len: usize = 1;

        #[inline]
        pub const fn $getter(&self) -> u8 {
            self.0[Self::$offset]
        }

        #[inline]
        pub fn $setter(&mut self, $getter: u8) {
            self.0[Self::$offset] = $getter;
        }

        #[inline]
        pub const fn $with(mut self, $getter: u8) -> Self {
            self.0[Self::$offset] = $getter;
            self
        }
    };
    ($getter:ident, $setter:ident, $with:ident, $offset:ident = $offset_value:expr, $len:ident = 2) => {
        const $offset: usize = $offset_value;
        const $len: usize = 2;

        #[inline]
        pub const fn $getter(&self) -> [u8; Self::$len] {
            [self.0[Self::$offset + 0], self.0[Self::$offset + 1]]
        }

        #[inline]
        pub fn $setter(&mut self, $getter: [u8; Self::$len]) {
            self.0[Self::$offset + 0] = $getter[0];
            self.0[Self::$offset + 1] = $getter[1];
        }

        #[inline]
        pub const fn $with(mut self, $getter: [u8; Self::$len]) -> Self {
            self.0[Self::$offset + 0] = $getter[0];
            self.0[Self::$offset + 1] = $getter[1];
            self
        }
    };
    ($getter:ident, $setter:ident, $with:ident, $offset:ident = $offset_value:expr, $len:ident = 4) => {
        const $offset: usize = $offset_value;
        const $len: usize = 4;

        #[inline]
        pub const fn $getter(&self) -> [u8; Self::$len] {
            [
                self.0[Self::$offset + 0],
                self.0[Self::$offset + 1],
                self.0[Self::$offset + 2],
                self.0[Self::$offset + 3],
            ]
        }

        #[inline]
        pub fn $setter(&mut self, $getter: [u8; Self::$len]) {
            self.0[Self::$offset + 0] = $getter[0];
            self.0[Self::$offset + 1] = $getter[1];
            self.0[Self::$offset + 2] = $getter[2];
            self.0[Self::$offset + 3] = $getter[3];
        }

        #[inline]
        pub const fn $with(mut self, $getter: [u8; Self::$len]) -> Self {
            self.0[Self::$offset + 0] = $getter[0];
            self.0[Self::$offset + 1] = $getter[1];
            self.0[Self::$offset + 2] = $getter[2];
            self.0[Self::$offset + 3] = $getter[3];
            self
        }
    };
}

#[allow(unused)]
pub mod data;
#[allow(unused)]
pub mod msg;
