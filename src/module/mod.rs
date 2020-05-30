pub mod ecm;

use crate::platform::Platform;
use crate::Action;
use crate::HandleError;
use core::convert::Infallible;
use sensor_common::Read;
use sensor_common::Request;
use sensor_common::Write;

pub trait RequestHandler {
    fn try_handle_request(
        &mut self,
        platform: &mut Platform,
        request: Request,
        request_payload: &mut impl sensor_common::Read,
        response_writer: &mut impl sensor_common::Write,
    ) -> Result<Action, HandleError>;
}

impl RequestHandler for Infallible {
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
