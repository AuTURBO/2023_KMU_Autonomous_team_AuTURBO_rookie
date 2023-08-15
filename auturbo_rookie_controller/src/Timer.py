#! /usr/bin/env python
# -*- coding:utf-8 -*-

import time

class Timer(object):
    '''
    Timer Class for speed optimization
    '''
    def __init__(self):
        # 초기시간 설정
        self.t0 = time.time()

    def update(self):
        '''
        update reference time
        '''
        # 초기시간 시간 업데이트
        self.t0 = time.time()

    def __call__(self):
        '''
        return duration time since reference time
        '''
        # 현재시간 - 초기시간
        return time.time() - self.t0
