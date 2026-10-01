@echo off
chcp 65001 > nul
cd /d %~dp0
if not exist .venv (
  python -m venv .venv
)
call .venv\Scripts\activate.bat
python -m pip install --upgrade pip
pip install -r requirements.txt
python run_experiment.py
python tests\test_reproducibility.py
echo.
echo Готово. Результаты находятся в папках figures и results.
pause
