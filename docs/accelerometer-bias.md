# Accelerometer bias: idle window and correction

The accelerometer (drive analog input 2, `ai2_value_V`) has an offset of about 0.65 V at rest, and that offset changes from experiment to experiment. Every run therefore measures its own offset before the excitation starts.

## What the firmware does

1. **Idle window.** For the first `BIAS_IDLE_S` (3 s) of every run the drive is enabled and commanded 0 A, so the mover is at rest. The electrical conditions are the same as during the run, with the drive switching. This happens in every experiment mode.
2. **Time axis.** The idle window has negative time, from `time_s` = -3 to 0. The excitation still starts at t = 0, so chirp and PRBS signals and their timestamps are identical to runs made before this change.
3. **Correction at export.** `export_csv()` takes the mean of `ai2_value_V` over all rows with t < 0. It prints the bias, the number of samples and the standard deviation on the console, and appends the column `ai2_corrected_V` = `ai2_value_V` − bias as the last column of the CSV.
   - `ai2_value_V` is kept unchanged, so the bias can always be recovered as `ai2_value_V - ai2_corrected_V`.
   - If there are no idle samples (for example a fault before t = 0), a warning is printed and `ai2_corrected_V` is `nan`.

Check the console line `AI2 bias: ... (std ...)` at the bench. A standard deviation much larger than usual means the mover or the table was disturbed during the idle window.

## How the analysis handles old and new logs

Both notebooks load logs through `vca_log.load_log(path)`. It returns a `VcaLog` with these fields:
- `run`: the experiment rows (t ≥ 0). It always has an `ai2_corrected_V` column.
- `idle`: the 0 A idle rows.
- `bias_V`: the bias that was removed.
- `bias_source`: where the bias came from.
- `idle_std_V`: the standard deviation of the accelerometer over the idle window.

| Log | `idle` | Bias taken from | `bias_source` |
|---|---|---|---|
| New (has idle window) | the t < 0 rows | the firmware's idle-window mean | `idle window` |
| Old, or new with `nan` column | empty | whole-run mean of `ai2_value_V` (the previous method) | `whole-run mean` |

- Old logs give exactly the same numbers as before this change.
- The loader also drops the duplicated first timestamp that the firmware writes.
- `vca_log.bias_report(logs, ACC_SCALE_V_PER_G)` makes a table of bias per run, which shows how the bias drifts between experiments.
- Wherever a log has an idle window, the notebooks also use it as the true zero-current noise floor:
  - in the system identification notebook, the Phase 1 noise spectra;
  - in the in-band notebook, section 7.

`scripts/plot_voice_coil_log.py` uses `ai2_corrected_V` when it is present. It shades the idle window in the plots and leaves it out of the printed averages and energy.

## Where to change things

| What | Where |
|---|---|
| Idle window length | `BIAS_IDLE_S` in `run_settings.h` |
| 0 A setpoint during the window | `fieldbus_run_cyclic()` in `control_loop.c` |
| Bias calculation and CSV column | `compute_ai2_bias_V()` and `export_csv()` in `logging.c` |
| Loading both log formats | `vca_log.py` |
