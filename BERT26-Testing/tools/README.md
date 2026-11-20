# tools

Python scripts for decoding and analysing BERT26 VCU logs (moved from `DataLoging/` in BERT26-Firmware_LEGACY). Behaviour is unchanged; each script's docstring says what it reads, writes and how to run it.

Install (only the two plot scripts need it): `python -m pip install -r requirements.txt`

| Script | What it does | How to run |
|---|---|---|
| `decoders/decode_vcu_log.py` | Decodes a `VCULOG01` session log into `_fast`, `_medium`, `_slow` (and `_imu`) csv files next to the input. | `python decoders/decode_vcu_log.py LOG0001.BIN` or `... --hex dump.txt` |
| `decoders/decode_cani.py` | Decodes a raw CAN log (`CANInnnn.BIN`) into `<name>_can.csv`. | `python decoders/decode_cani.py CANI0012.BIN` |
| `decoders/samples/` | `LOG0001.BIN` (11 s, from the 1st set of runs) with its decoded csvs, to smoke-test the decoder. | `python decoders/decode_vcu_log.py decoders/samples/LOG0001.BIN` (output should match the csvs there) |
| `analysis/analyze_runs.py` | Per log: max power, max V*I, calculated vs commanded torque, max rpm, BSPD-low windows. | `python analysis/analyze_runs.py <folder of LOG*.BIN>` |
| `analysis/analyze_spread.py` | Cell-voltage spread per session and its correlation with bus current. | `python analysis/analyze_spread.py` (reads csvs from `tools/`, `tools/3rd set of runs/`, `tools/4th Set of runs/` relative to the working directory) |
| `analysis/spread_detail.py` | Spread detail for LOG0063-0066 (low/high-side deviation, trend, idle vs load). | `python analysis/spread_detail.py` (reads `tools/4th Set of runs/` relative to the working directory) |
| `analysis/bspd_detail.py` | Max calculated torque above 1000 rpm and context around three BSPD-low events. | `python analysis/bspd_detail.py` (reads `3rd set of runs/` relative to the working directory) |
| `analysis/find_tsv_over100.py` | Finds spans where `bus_voltage_v` stays above 100 V for more than 45 s. | `python analysis/find_tsv_over100.py [folder of *_fast.csv]` |
| `analysis/plot_log12.py` | LOG0012 spread-vs-time and min/avg/max cell plots; writes two PNGs. | `python analysis/plot_log12.py` (needs `LOG0012_slow.csv` in the working directory) |
| `analysis/plot_log12_sag.py` | LOG0012 measured vs expected (13 mOhm DCIR, 4P) cell sag; writes one PNG. | `python analysis/plot_log12_sag.py` (needs `LOG0012_slow.csv` and `LOG0012_fast.csv` in the working directory) |
| `scripts/` | Empty for now. | |

Known quirks kept from the originals: `plot_log12_sag.py` prints "load > 20 A" but uses 30 A; its plot annotation indexes the load-only residual array with a full-array index; `analyze_runs.py` needs its folder argument.
