# License overview

libccamera is licensed under the GNU Lesser General Public License, version 2.1
or (at your option) any later version (`LGPL-2.1-or-later`). The top-level
[LICENSE](../LICENSE) contains the LGPL 2.1 text. Individual file notices take
precedence over this overview.

- Library code, public headers, examples and most tests use
  [LGPL-2.1-or-later](LGPL-2.1-or-later.txt).
- Files derived from Raspberry Pi's libcamera/libpisp code and calibration
  retain their [BSD-2-Clause](BSD-2-Clause.txt) notices. Some files require both
  LGPL and BSD terms, as indicated by their SPDX `AND` expression.
- Vendored Linux UAPI headers retain their file-specific licenses, including
  [GPL-2.0-only](GPL-2.0-only.txt) with the
  [Linux-syscall-note](Linux-syscall-note.txt) exception, and the alternative
  [BSD-3-Clause](BSD-3-Clause.txt) terms where indicated.
- Build helpers and GitHub configuration explicitly marked `CC0-1.0` use
  [CC0 1.0](CC0-1.0.txt).
- The OV5647 calibration data has a separate
  [license notice](../data/ov5647-vc4.json.license), so the JSON remains unchanged.

The external upstream reference archive is not distributed with this project.
