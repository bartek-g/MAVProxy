#!/usr/bin/env python
'''mode command handling'''

import time, os
from pymavlink import mavutil

from MAVProxy.modules.lib import mp_module

class ModeModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(ModeModule, self).__init__(mpstate, "mode")
        self.add_command('mode', self.cmd_mode, "mode change", self.available_modes())
        self.add_command('guided', self.cmd_guided, "fly to a clicked location on map")
        self.add_command('guided_loiter', self.cmd_guided_loiter, "fly to a clicked location and loiter")

    def cmd_mode(self, args):
        '''set arbitrary mode'''
        mode_mapping = self.master.mode_mapping()
        if mode_mapping is None:
            print('No mode mapping available')
            return
        if len(args) != 1:
            print('Available modes: ', mode_mapping.keys())
            return
        if args[0].isdigit():
            modenum = int(args[0])
        else:
            mode = args[0].upper()
            if mode not in mode_mapping:
                print('Unknown mode %s: ' % mode)
                return
            modenum = mode_mapping[mode]
        self.master.set_mode(modenum)

    def available_modes(self):
        mode_mapping = self.master.mode_mapping()
        if mode_mapping is None:
            print('No mode mapping available')
            return []
        return mode_mapping.keys()

    def unknown_command(self, args):
        '''handle mode switch by mode name as command'''
        mode_mapping = self.master.mode_mapping()
        mode = args[0].upper()
        if mode in mode_mapping:
            self.master.set_mode(mode_mapping[mode])
            return True
        return False

    def cmd_guided(self, args):
        '''set GUIDED target'''
        arg_count = len(args)
        if arg_count != 1 and arg_count != 3 and arg_count != 4:
            print("Usage: guided ALTITUDE | guided LAT LON ALTITUDE | guided LAT LON ALTITUDE RADIUS")
            return

        using_guided_loiter = False
        if len(args) == 4:
            latitude = float(args[0])
            longitude = float(args[1])
            altitude = float(args[2])
            loiter_radius = float(args[3])
            lat_lon = (latitude, longitude)
            using_guided_loiter = True
        elif len(args) == 3:
            latitude = float(args[0])
            longitude = float(args[1])
            altitude = float(args[2])
            lat_lon = (latitude, longitude)
        else:
            try:
                lat_lon = self.module('map').click_position
            except Exception:
                print("No map available")
                return
            if lat_lon is None:
                print("No map click position available")
                return
            altitude = float(args[0])

        if not using_guided_loiter:
            print("Guided %s %s" % (str(lat_lon), str(altitude)))
            self.master.mav.mission_item_send (self.settings.target_system,
                                               self.settings.target_component,
                                               0,
                                               self.module('wp').get_default_frame(),
                                               mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                                               2, 0, 0, 0, 0, 0,
                                               lat_lon[0], lat_lon[1], altitude)
        else:
            print("Guided loitering %s %s radius %s" % (str(lat_lon), str(altitude), str(loiter_radius)))
            self.master.mav.mission_item_send (self.settings.target_system,
                                               self.settings.target_component,
                                               0,
                                               self.module('wp').get_default_frame(),
                                               mavutil.mavlink.MAV_CMD_NAV_LOITER_TURNS,
                                               2, 0, 0, 0, loiter_radius, 0,
                                               lat_lon[0], lat_lon[1], altitude)

    def cmd_guided_loiter(self, args):
        arg_count = len(args)
        if arg_count != 1:
            print("Usage: guided_loiter RADIUS")
            return
        try:
            lat_lon = self.module('map').click_position
        except Exception:
            print("No map available")
            return
        altitude = self.mpstate.master().field('GLOBAL_POSITION_INT', 'relative_alt', 0) * 1.0e-3
        loiter_radius = float(args[0])
        print("Guided loitering %s %s radius %s" % (str(lat_lon), str(altitude), str(loiter_radius)))
        self.master.mav.mission_item_send(self.settings.target_system,
                                          self.settings.target_component,
                                          0,
                                          self.module('wp').get_default_frame(),
                                          mavutil.mavlink.MAV_CMD_NAV_LOITER_TURNS,
                                          2, 0, 0, 0, loiter_radius, 0,
                                          lat_lon[0], lat_lon[1], altitude)


def init(mpstate):
    '''initialise module'''
    return ModeModule(mpstate)
