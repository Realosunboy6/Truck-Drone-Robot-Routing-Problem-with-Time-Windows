# Push Folder Manifest

This folder is a clean GitHub-ready copy of the final capped flexible-docking ordered-sortie model package.

Included:
- `model/`: final capped flexible-docking ordered-sortie MILP model only.
- `scripts/`: data builder, batch runner, and result summarizer for the strict capped model.
- `data_raw/`: source raw TDRP-TW data used for traceability.
- `data_processed/tdrp_tw_literature_params/`: the 18 processed benchmark cases used by the model.
- `results/final_5min/`: official 5-minute capped-model run.
- `results/final_30min/`: official 30-minute capped-model run.
- `parameter_usage_audit.csv`: parameter usage audit.
- `pdf/`: `model_formulation.tex` and compiled `model_formulation.pdf`, the MILP
  formulation written to match `model/capped_flexible_docking_ordered_sortie_model.py`;
  plus `drone_robot_truck_math_model.pdf`, the reference math-model PDF used for
  formulation traceability.
- `README.md` and `requirements.txt`.

Each result run is organized into:
- `workbooks/`: Excel solution workbooks.
- `summaries/`: extracted summary CSVs and timing audits.
- `lp_models/`: exported LP model files, except the two `11-25` LP exports
  that exceed GitHub's 100 MB file-size limit and are kept local-only.
- `logs/`: console and batch logs.
- `model_notes/`: generated model-note files.
- `infeasibility_notes/`: notes for cases that did not produce a usable solution.
- `batch_status/`: per-case batch status CSV.

Excluded / moved out:
- non-report experiment folders
- manual rerun folders
- Python cache files
