/PROG  H002.LS
/ATTR
OWNER       = MNEDITOR;
COMMENT     = "";
PROG_SIZE   = 636;
CREATE      = DATE 23-01-07  TIME 11:59:14;
MODIFIED    = DATE 23-01-07  TIME 12:02:18;
FILE_NAME   = ;
VERSION     = 0;
LINE_COUNT  = 4;
MEMORY_SIZE = 992;
PROTECT     = READ_WRITE;
TCD:  STACK_SIZE    = 0,
      TASK_PRIORITY = 50,
      TIME_SLICE    = 0,
      BUSY_LAMP_OFF = 0,
      ABORT_REQUEST = 0,
      PAUSE_REQUEST = 0;
DEFAULT_GROUP    = 1,*,*,*,*;
CONTROL_CODE     = 00000000 00000000;
/MN
   1:J P[1] 100% FINE    ;
   2:L P[2] 100mm/sec CNT100    ;
   3:L P[3] 100mm/sec CNT100    ;
   4:L P[4] 100mm/sec CNT100    ;
   5:L P[5] 100mm/sec CNT100    ;
   6:L P[6] 100mm/sec CNT100    ;
   7:L P[7] 100mm/sec CNT100    ;
/POS
P[1]{
   GP1:
    UF : 0, UT : 6,      CONFIG : 'N U T, 0, 0, 0',
    X =  420.246  mm,    Y =   0.000  mm,    Z =   53.417  mm,
    W =  -180.000 deg,    P =   0.000 deg,    R =   0.000 deg
};
P[2]{
   GP1:
    UF : 0, UT : 6,      CONFIG : 'N U T, 0, 0, 0',
    X =  583.845  mm,    Y =   125.748  mm,    Z =   -175.827  mm,
    W =  -180.000 deg,    P =   0.000 deg,    R =   0.000 deg
};
P[3]{
   GP1:
    UF : 0, UT : 6,      CONFIG : 'N U T, 0, 0, 0',
    X =  470.156  mm,    Y =   125.748  mm,    Z =   -175.827  mm,
    W =  -180.000 deg,    P =   0.000 deg,    R =   0.000 deg
};
P[4]{
   GP1:
    UF : 0, UT : 6,      CONFIG : 'N U T, 0, 0, 0',
    X =  470.156  mm,    Y =   -144.576  mm,    Z =   -175.827  mm,
    W =  -180.000 deg,    P =   0.000 deg,    R =   0.000 deg
};
P[5]{
   GP1:
    UF : 0, UT : 6,      CONFIG : 'N U T, 0, 0, 0',
    X =  583.845  mm,    Y =   -144.576  mm,    Z =   -175.827  mm,
    W =  -180.000 deg,    P =   0.000 deg,    R =   0.000 deg
};
P[6]{
   GP1:
    UF : 0, UT : 6,      CONFIG : 'N U T, 0, 0, 0',
    X =  583.845  mm,    Y =   125.748  mm,    Z =   -175.827  mm,
    W =  -180.000 deg,    P =   0.000 deg,    R =   0.000 deg
};
P[7]{
   GP1:
    UF : 0, UT : 6,      CONFIG : 'N U T, 0, 0, 0',
    X =  420.246  mm,    Y =   0.000  mm,    Z =   53.417  mm,
    W =  -180.000 deg,    P =   0.000 deg,    R =   0.000 deg
};
/END
