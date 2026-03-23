@echo off
echo ========================================================
echo Willow 5 Runtime - Automated Deployment Script
echo ========================================================

:: Define the strict target repository
set TARGET_REPO=https://github.com/Willow-Dynamics/willow-cpp-sdk.git
set BRANCH=main

:: 1. Ensure local git is initialized
git init

:: 2. Wipe any existing origin connections to prevent cross-repo contamination
git remote remove origin 2>nul

:: 3. Establish the strictly routed connection
echo [Network] Establishing strict connection to %TARGET_REPO%...
git remote add origin %TARGET_REPO%

:: 4. Stage all un-ignored files
echo [System] Staging SDK files...
git add .

:: 5. Commit with an automated timestamp
echo [System] Committing deployment...
git commit -m "v5.2.1 - Math kernel optimizations. - Dynamic zone masking added for hand tracking fidelity improvements."

:: 6. Enforce branch name
git branch -M %BRANCH%

:: 7. Push to the remote
echo [Network] Pushing SDK to GitHub...
git push -u origin %BRANCH%

echo.
echo ========================================================
echo Deployment Complete! The SDK is live.
echo ========================================================
pause