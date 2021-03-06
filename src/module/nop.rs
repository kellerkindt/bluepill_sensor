use crate::module::{
    Module, ModuleBuilder, ModulePeripherals, PlatformConstraints, RequestHandler,
};
use crate::platform::{Action, HandleError, Platform};
use sensor_common::props::{ModuleId, Property};
use sensor_common::{Read, Request, Write};

pub struct NopModuleBuilder;

impl ModuleBuilder<NopModule> for NopModuleBuilder {
    fn build(
        _platform: &mut Platform,
        _constraints: &mut PlatformConstraints,
        _peripherals: ModulePeripherals,
    ) -> NopModule {
        NopModule
    }
}

pub struct NopModule;

impl Module for NopModule {
    type Builder = NopModuleBuilder;
    const PROPERTIES: &'static [Property<Platform, Self>] = &[];

    fn module_id(&self) -> ModuleId {
        ModuleId {
            group: 0,
            id: 0,
            ext: 0,
        }
    }

    fn update(&mut self, _platform: &mut Platform) {}
}

impl RequestHandler for NopModule {
    fn try_handle_request(
        &mut self,
        _platform: &mut Platform,
        request: Request,
        _request_payload: &mut impl Read,
        _response_writer: &mut impl Write,
    ) -> Result<Action, HandleError> {
        Ok(Action::HandleRequest(request))
    }
}
