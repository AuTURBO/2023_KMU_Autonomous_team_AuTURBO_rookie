#! /usr/bin/env python
# -*- coding:utf-8 -*-

import time

class Timer(object):
    '''
    Timer Class for speed optimization
    '''
    def __init__(self):
        self.t0 = time.time()

    def update(self):
        '''
        update reference time
        '''
        self.t0 = time.time()

    def __call__(self):
        '''
        return duration time since reference time
        '''
        return time.time() - self.t0