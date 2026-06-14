# Results Folder

This folder contains only the official capped flexible-docking ordered-sortie runs used for reporting.

## Runs

- `final_5min/`: 18-case run with 300 seconds per case.
- `final_30min/`: 18-case run with 1800 seconds per case.

## Subfolders inside each run

- `workbooks/`: Excel solution workbooks for each case.
- `summaries/`: extracted result tables and customer timing audits.
- `lp_models/`: exported LP model files for traceability. The two `11-25`
  LP exports are intentionally kept local-only because each exceeds GitHub's
  100 MB file-size limit.
- `logs/`: console and batch logs.
- `model_notes/`: generated notes describing model assumptions and selected solution structure.
- `infeasibility_notes/`: notes for cases that did not produce a usable solution.
- `batch_status/`: batch status CSV showing start/end time and exit code per case.

Recommended files to inspect first:

- `final_5min/summaries/customer_windows_service_assignment_audit.csv`
- `final_5min/summaries/case_status_cost_summary.csv`
- `final_30min/summaries/customer_windows_service_assignment_audit_30min.csv`
- `final_30min/summaries/case_status_cost_summary_30min.csv`

Note: in the 30-minute run, case `11-25` did not produce a usable feasible solution because CPLEX ran out of memory. Its explanation is stored in `final_30min/infeasibility_notes/`.
