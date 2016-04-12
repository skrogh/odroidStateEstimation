set terminal x11 noraise
set view equal xyz
set size ratio -1

p "< tail -400 test" u 1:2 w l

pause 0.25
reread
