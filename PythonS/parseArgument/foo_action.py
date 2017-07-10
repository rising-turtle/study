#!/usr/bin/env python

import argparse

class FooAction(argparse.Action):
    def __init__(self, option_strings, dest, nargs=None, **kwargs):
        if nargs is not None:
            raise ValueError("nargs not allowed!")
        super(FooAction, self).__init__(option_strings, dest, **kwargs)
    def __call__(self, parser, namespace, values, option_strings=None):
        print('%r %r %r' % (namespace, values, option_strings))
        setattr(namespace, self.dest, values)

