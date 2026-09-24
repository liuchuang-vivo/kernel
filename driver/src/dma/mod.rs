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

//! DMA controller drivers.
//!
//! SoC-specific implementations are selected by `#[cfg(soc_*)]` at the
//! module boundary, mirroring the `i2c` / `spi` / `uart` siblings.

#[cfg(soc_esp32c6)]
pub mod esp32c6_gdma;
