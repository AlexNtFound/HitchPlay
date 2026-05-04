@echo off
REM ----------------------------------------------------------------------
REM  Ursa build entry point (Windows).
REM
REM  Auto-detects a JDK so collaborators don't need to set JAVA_HOME by hand,
REM  then forwards all args to gradlew.bat. Resolution order:
REM    1) JAVA_HOME, if already set and valid
REM    2) Android Studio's bundled JBR (standard install paths)
REM    3) Bail with an actionable message
REM ----------------------------------------------------------------------
setlocal

set "VALID="

if defined JAVA_HOME (
    if exist "%JAVA_HOME%\bin\java.exe" set "VALID=1"
)

if not defined VALID (
    for %%P in (
        "%ProgramFiles%\Android\Android Studio\jbr"
        "%LOCALAPPDATA%\Programs\Android Studio\jbr"
        "%ProgramFiles(x86)%\Android\Android Studio\jbr"
        "%ProgramFiles%\Eclipse Adoptium\jdk-17"
        "%ProgramFiles%\Eclipse Adoptium\jdk-21"
        "%ProgramFiles%\Java\jdk-17"
        "%ProgramFiles%\Java\jdk-21"
    ) do (
        if not defined VALID if exist "%%~P\bin\java.exe" (
            set "JAVA_HOME=%%~P"
            set "VALID=1"
        )
    )
)

if not defined VALID (
    echo.
    echo [Ursa] Could not find a JDK to run the Gradle wrapper.
    echo.
    echo Easy fix: install Android Studio (its bundled JBR is auto-detected),
    echo or install OpenJDK 17 from https://adoptium.net/temurin/releases/?version=17
    echo and set JAVA_HOME to point at it.
    echo.
    exit /b 1
)

echo [Ursa] Using JAVA_HOME=%JAVA_HOME%
"%~dp0gradlew.bat" %*
