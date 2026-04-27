@echo off
rem Reset current branch to origin/<branch> and clean working tree
rem WARNING: this will discard local changes and untracked files

  for /f "delims=" %%b in ('git rev-parse --abbrev-ref HEAD') do set "BRANCH=%%b"
echo Current branch: %BRANCH%

git fetch --all --prune
if errorlevel 1 (
  echo git fetch failed & exit /b 1
)

git reset --hard origin/%BRANCH%
if errorlevel 1 (
  echo git reset failed & exit /b 1
)

git clean -fdx
if errorlevel 1 (
  echo git clean failed & exit /b 1
)

echo Done. Repository reset to origin/%BRANCH% and cleaned.
