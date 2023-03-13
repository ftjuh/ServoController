@echo off
REM Provides an rm command under Windows.

set tmp=%*
set tmp=%tmp:/=\%
set tmp=%tmp:-f = %

del /Q %tmp%
