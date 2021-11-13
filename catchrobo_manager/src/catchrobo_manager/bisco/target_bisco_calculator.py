#!/usr/bin/env python
# -*- coding: utf-8 -*-

import pandas as pd
import numpy as np
import copy


class TargetBiscoCalculator:

    # [TODO]
    def calcTargetTwin(self, database):
        biscos = copy.deepcopy(database)

        # our area
        in1 = in2 = in3 = in4 = in5 = in6 = 0
        count=0
        for i in range(6):
            # represents the state of biscos next to each other.
            val = 2*biscos.isExist(26-i-6)+biscos.isExist(26-i)

            if val == 2:  # ox
                if i >= 0 and i <= 1:
                    biscos.updateState(26-i-12, "priority",biscos.getState(26-i-12, "priority")-20)
                else:
                    biscos.updateState(26-i-12, "priority",biscos.getState(26-i-12, "priority")-20)

            if val == 1:  # xo
                biscos.updateState(26-i, "priority", biscos.getState(26-i, "priority")+26)
                
                if biscos.getState(26, "priority") == 27 and in1 == 0:
                    count = 1
                    in1 = 1
                    biscos.updateState(24, "priority", 5)
                    biscos.updateState(18, "priority", 6)
                    for k in range(8):
                        biscos.updateState(k+1, "priority", biscos.getState(k+1, "priority")+2)
                    #print("count=1\n")

                if biscos.getState(25, "priority") == 29 and in2 == 0:
                    in2 = 1
                    if count == 0:  # have to get 2 more
                        count = 2
                        biscos.updateState(24, "priority", 3)
                        biscos.updateState(18, "priority", 4)
                        #print("count=2\n")
                    elif count == 1:  # have to get all 4
                        count = 3
                        biscos.updateState(23, "priority", 7)
                        biscos.updateState(17, "priority", 8)
                        for k in range(8):
                            biscos.updateState(k+1, "priority", biscos.getState(k+1, "priority")+2)
                        #print("count=3\n")

                if (biscos.getState(24, "priority") == 29 or biscos.getState(24, "priority") == 31) and in3 == 0:
                    in3 = 1
                    if count == 1:
                        count = 4
                        biscos.updateState(23, "priority", 5)
                        biscos.updateState(17, "priority", 6)
                        #print("count=4\n")

                    elif count == 2:
                        count = 5
                        biscos.updateState(23, "priority", 3)
                        biscos.updateState(17, "priority", 4)
                        biscos.updateState(25, "priority", 28)
                        #print("count=5\n")

                    elif count == 3:
                        count = 6
                        biscos.updateState(23, "priority", 1)
                        biscos.updateState(17, "priority", 2)
                        biscos.updateState(22, "priority", 3)
                        biscos.updateState(16, "priority", 4)
                        #print("count=6\n")

                if (biscos.getState(23, "priority") == 27 or biscos.getState(23, "priority") == 29 or biscos.getState(23, "priority") == 31 or biscos.getState(23, "priority") == 33) and in4 == 0:
                    in4 = 1
                    if count == 3:
                        count = 7
                        biscos.updateState(22, "priority", 7)
                        biscos.updateState(16, "priority", 8)
                        #print("count=7\n")

                    elif count == 4:
                        count = 8
                        biscos.updateState(22, "priority", 5)
                        biscos.updateState(16, "priority", 6)
                        biscos.updateState(24, "priority", 30)
                        #print("count=8\n")

                    elif count == 5:
                        count = 9
                        biscos.updateState(22, "priority", 3)
                        biscos.updateState(16, "priority", 4)
                        biscos.updateState(23, "priority", 31)
                        #print("count=9\n")

                    elif count == 6:
                        count = 10
                        biscos.updateState(21, "priority", 5)
                        biscos.updateState(15, "priority", 6)
                        biscos.updateState(23, "priority", 33)
                        #print("count=10\n")

                if (biscos.getState(22, "priority") == 29 or biscos.getState(22, "priority") == 31 or biscos.getState(22, "priority") == 33) and in5 == 0:
                    in5 = 1
                    if count == 6:
                        count = 11
                        biscos.updateState(21, "priority", 3)
                        biscos.updateState(15, "priority", 4)
                        biscos.updateState(22, "priority", 33)
                        #print("count=11\n")

                    elif count == 7:
                        count = 12
                        biscos.updateState(21, "priority", 7)
                        biscos.updateState(15, "priority", 8)
                        biscos.updateState(22, "priority", 35)
                        #print("count=12\n")

                    elif count == 8:
                        count = 13
                        biscos.updateState(21, "priority", 5)
                        biscos.updateState(15, "priority", 6)
                        biscos.updateState(22, "priority", 33)
                        #print("count=13\n")

                    elif count == 9:
                        count = 14
                        biscos.updateState(21, "priority", 3)
                        biscos.updateState(15, "priority", 4)
                        biscos.updateState(22, "priority", 33)
                        #print("count=14\n")

                    elif count == 10:  # not twin is better
                        count = 15
                        biscos.updateState(26, "priority", 7)
                        biscos.updateState(22, "priority", 35)
                        #print("count=15\n")

                if (biscos.getState(21, "priority") == 29 or biscos.getState(21, "priority") == 31 or biscos.getState(21, "priority") == 33) and in6 == 0:
                    in6 = 1
                    if count == 10:# not twin is better
                        count = 16
                        biscos.updateState(26, "priority", 7)
                        biscos.updateState(21, "priority", 35)
                        #print("count=16\n")

                    elif count == 11:# not twin is better
                        count = 17
                        biscos.updateState(26, "priority", 7)
                        biscos.updateState(21, "priority", 35)
                        #print("count=17\n")

                    elif count == 12:# not twin is better
                        count = 18
                        biscos.updateState(26, "priority", 7)
                        biscos.updateState(21, "priority", 37)
                        #print("count=18\n")

                    elif count == 13:# not twin is better
                        count = 19
                        biscos.updateState(26, "priority", 5)
                        biscos.updateState(21, "priority", 35)
                        #print("count=19\n")

                    elif count == 14:  # not twin is better
                        count = 20
                        biscos.updateState(25, "priority", 3)
                        biscos.updateState(21, "priority", 35)
                        #print("count=20\n")

                    elif count == 15:  # not twin is better
                        count = 21
                        biscos.updateState(26, "priority", 1)
                        biscos.updateState(25, "priority", 2)
                        biscos.updateState(24, "priority", 3)
                        biscos.updateState(21, "priority", 37)
                        #print("count=21\n")
        

        # common area        
        for j in range(6):
            check=2*biscos.isExist(7-j)+biscos.isExist(6-j)
            #print(check)
            if check==2:
                biscos.updateState(7-j, "priority", biscos.isExist(7-j)+40)
        
        '''
        # debag
        print("in1=",in1)
        print("in2=",in2)
        print("in3=",in3)
        print("in4=",in4)
        print("in5=",in5)
        print("in6=",in6)
        for h in range(6):
            print(biscos.getState(8-h, "priority"), biscos.getState(14-h, "priority"),biscos.getState(20-h, "priority"), biscos.getState(26-h, "priority"))
        print("\n")
        '''

        first = self.getMininumPriorityId(biscos)
        if first is None:
            return None, None

        biscos.delete(first)
        second = self.getMininumPriorityId(biscos)
        if biscos.isExsist(second)==0:
            return first, None
        
        return first, second

    def getMininumPriorityId(self, database):
        exist = database._objects["exist"]
        # print(self._objects[exist]["priority"])
        if np.sum(exist) == 0:
            minimum = None
        else:
            minimum = database._objects[exist]["priority"].idxmin()
        return minimum

    def isNeighbor(self, biscos, first, second):
        ret = False
        if first is None or second is None:
            return False
        areas = [biscos.getState(first, "my_area"),
                 biscos.getState(second, "my_area")]
        if areas[0] == areas[1] == True:
            if abs(first - second) == 6:
                ret = True
        elif areas[0] == areas[1] == False:
            if abs(first - second) == 1:
                ret = True
        return ret


'''
num

08 14 20 26
07 13 19 25
06 12 18 24
05 11 17 23
04 10 16 22
03 09 15 21
02
01

priority count=0

99 21 02 01 
05 22 04 03
06 23 14 13
07 24 16 15
08 25 18 17
09 26 20 19
10 
11
12

priority count=1
05 06 +2

99 21 xx 27 
07 22 04 03
08 23 06 05
09 24 16 15
10 25 18 17
11 26 20 19
12 
13
14

priority count=2
03 04

99 21 02 01 
05 22 xx 29
06 23 04 03
07 24 16 15
08 25 18 17
09 26 20 19
10 
11
12

priority count=3
07 08 +2

99 21 xx 27 
09 22 xx 29
10 23 06 05
11 24 08 07
12 25 18 17
13 26 20 19
14 
15
16

priority count=4
05 06

99 21 xx 27 
07 22 04 03
08 23 xx 31
09 24 06 05
10 25 18 17
11 26 20 19
12 
13
14

priority count=5
03 04 28

99 21 02 01 
05 22 xx 29(=28)
06 23 xx 29
07 24 04 03
08 25 18 17
09 26 20 19
10 
11
12

priority count=6
01 02 03 04

99 21 xx 27 
09 22 xx 29
10 23 xx 31
11 24 02 01
12 25 04 03
13 26 20 19
14 
15
16



priority count=7
07 08

99 21 xx 27 
09 22 xx 29
10 23 06 05
11 24 xx 33
12 25 08 07
13 26 20 19
14 
15
16

priority count=8
05 06 30

99 21 xx 27 
07 22 04 03
08 23 xx 31(=30)
09 24 xx 31
10 25 06 05
11 26 20 19
12 
13
14

priority count=9
03 04 31

99 21 02 01 
05 22 xx 28
06 23 xx 29
07 24 xx 29(=31)
08 25 04 03
09 26 20 19
10 
11
12


priority count=10
05 06 33

99 21 xx 27 
09 22 xx 29
10 23 xx 31
11 24 xx 27(=33)
12 25 04 03
13 26 06 05
14 
15
16

priority count=11
03 04 33

99 21 xx 27 
09 22 xx 29
10 23 xx 31
11 24 02 01
12 25 xx 29(=33)
13 26 04 03
14 
15
16



priority count=12
07 08 35

99 21 xx 27 
09 22 xx 29
10 23 06 05
11 24 xx 33
12 25 xx 33(=35)
13 26 08 07
14 
15
16

priority count=13
05 06 33

99 21 xx 27 
07 22 04 03
08 23 xx 30
09 24 xx 31
10 25 xx 31(=33)
11 26 06 05
12 
13
14


priority count=14
03 04 33

99 21 02 01 
05 22 xx 28
06 23 xx 29
07 24 xx 31
08 25 xx 29(=33)
09 26 04 03
10 
11
12


priority count=15
07 35

99 21 xx 07 
09 22 xx 29
10 23 xx 31
11 24 xx 33
12 25 xx 29(=35)
13 26 06 05
14 
15
16

priority count=16
07 35

99 21 xx 07 
09 22 xx 29
10 23 xx 31
11 24 xx 33
12 25 04 03
13 26 xx 31(=35)
14 
15
16


priority count=17
07 35

99 21 xx 07 
09 22 xx 29
10 23 xx 31
11 24 02 01
12 25 xx 33
13 26 xx 29(=35)
14 
15
16


priority count=18
07 37

99 21 xx 07 
09 22 xx 29
10 23 06 05
11 24 xx 33
12 25 xx 35
13 26 xx 33(=37)
14 
15
16

priority count=19
05 35

99 21 xx 05 
07 22 04 03
08 23 xx 30
09 24 xx 31
10 25 xx 33
11 26 xx 31(=35)
12 
13
14


priority count=20
03 35

99 21 02 01 
05 22 xx 28(=03)
06 23 xx 29
07 24 xx 31
08 25 xx 33
09 26 xx 29(=35)
10 
11
12


priority count=21
01 02 03 37

99 21 xx 07(=01)
09 22 xx 29(=02)
10 23 xx 31(=03)
11 24 xx 33
12 25 xx 35
13 26 xx 31(=37)
14 
15
16

'''
