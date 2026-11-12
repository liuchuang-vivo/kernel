// Copyright (c) 2026 vivo Mobile Communication Co., Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//       http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//! Kernel host adapters for standalone image loading.
//!
//! The loader core stays independent of kernel types. This module supplies the
//! flat memory mapping, instruction publication and native profile, then hands
//! the owning `LoadedImage` back to the caller. Startup storage retains that
//! image and its ABI buffers. Application starts and observes the thread group,
//! while the application libc executes initialization and finalization.

pub mod adapters;
pub(crate) mod application;
mod load;
#[cfg(enable_vfs)]
mod run;
mod startup;

pub use adapters::flat_memory::init_fixed_regions;
pub use application::{Application, ApplicationError, ApplicationExit, ApplicationState};
pub use load::{board_load_profile, load_image, load_request};
#[cfg(enable_vfs)]
pub use run::{run, RunError, RunRequest};
pub use startup::{StartupArguments, StartupError, StartupStorage};
