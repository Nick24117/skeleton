#!/usr/bin/env python

# TODO: openbmc/openbmc#2994 remove python 2 support
try:  # python 2
    import gobject
except ImportError:  # python 3
    from gi.repository import GObject as gobject
import dbus
import dbus.service
import dbus.mainloop.glib
import os
from obmc.dbuslib.bindings import DbusProperties, DbusObjectManager, get_dbus
import obmc.enums
import obmc_system_config as System
import obmc.inventory
import obmc.system
import subprocess

DBUS_NAME = 'org.openbmc.managers.System'
OBJ_NAME = '/org/openbmc/managers/System'
INTF_SENSOR = 'org.openbmc.SensorValue'

class SystemManager(DbusProperties, DbusObjectManager):
    def __init__(self, bus, obj_name):
        super(SystemManager, self).__init__(
            conn=bus,
            object_path=obj_name)
        self.bus = bus

        # replace symbolic path in ID_LOOKUP
        for category in System.ID_LOOKUP:
            for key in System.ID_LOOKUP[category]:
                val = System.ID_LOOKUP[category][key]
                new_val = val.replace(
                    "<inventory_root>", obmc.inventory.INVENTORY_ROOT)
                System.ID_LOOKUP[category][key] = new_val

        print("SystemManager Init Done")

    def doObjectLookup(self, category, key):
        obj_path = ""
        intf_name = INTF_SENSOR
        try:
            obj_path = System.ID_LOOKUP[category][key]
            parts = obj_path.split('/')
            if (parts[3] != 'sensors'):
                print ("ERROR SystemManager: SENSOR only supported type")
                intf_name = ""
        except Exception as e:
            print ("ERROR SystemManager: "+str(e)+" not found in lookup")

        return [obj_path, intf_name]

    @dbus.service.method(DBUS_NAME, in_signature='ss', out_signature='(ss)')
    def getObjectFromId(self, category, key):
        return self.doObjectLookup(category, key)

    @dbus.service.method(DBUS_NAME, in_signature='sy', out_signature='(ss)')
    def getObjectFromByteId(self, category, key):
        byte = int(key)
        return self.doObjectLookup(category, byte)

    @dbus.service.method(DBUS_NAME,
        in_signature='s', out_signature='s')
    def convertHwmonPath(self, path):
        try:
            split_p = path.split(":")
            if len(split_p) < 2:
                return ""
            i2c_bus = split_p[0].split("-")[0]
            hwmon_i2c_path = "/sys/bus/i2c/devices/i2c-%s/%s/hwmon" % (i2c_bus, split_p[0])
            for dirname, dirnames, filenames in os.walk(hwmon_i2c_path):
                hwmon_path = dirname + "/" + split_p[1]
                if os.path.exists(hwmon_path):
                    return hwmon_path
        except:
            print "convertHwmonPath Error:" + path
        return ""

    @dbus.service.method(DBUS_NAME,
        in_signature='s', out_signature='s')
    def getFanControlParams(self, key):
        if ('FAN_ALGORITHM_CONFIG' not in dir(System) or key == None):
            return ""
        s_params = ""
        try:
            if key == "INVENTORY_FAN":
                for inventory_obj_path in System.FRU_INSTANCES:
                    if inventory_obj_path.find("fan") >= 0:
                        data = inventory_obj_path.replace("<inventory_root>", System.INVENTORY_ROOT)
                        s_params+=data + ";"
                        return s_params

            if key.find("FAN_DBUS_INTF_LOOKUP") >= 0:
                key_array = key.split("#")
                if len(key_array) != 2:
                    return ""
                key_name = key_array[0]
                key_prefix = key_array[1]
                if key_name not in System.FAN_ALGORITHM_CONFIG:
                    return ""
                if key_prefix not in System.FAN_ALGORITHM_CONFIG[key_name]:
                    return ""
                for i in range(len(System.FAN_ALGORITHM_CONFIG[key_name][key_prefix])):
                    s_params += System.FAN_ALGORITHM_CONFIG[key_name][key_prefix][i] + ";"
                return s_params

            if key not in System.FAN_ALGORITHM_CONFIG:
                return ""

            if key == "SET_FAN_OUTPUT_OBJ":
                for i in range(len(System.FAN_ALGORITHM_CONFIG[key])):
                    hwmon_path = self.convertHwmonPath(System.FAN_ALGORITHM_CONFIG[key][i])
                    s_params += hwmon_path + ";"
                return s_params

            i = 0
            while i<len(System.FAN_ALGORITHM_CONFIG[key]):
                if System.FAN_ALGORITHM_CONFIG[key][i].find("Sensor_Group_List") >= 0:
                    sensor_path_format = System.FAN_ALGORITHM_CONFIG[key][i+1]
                    sensor_amount_range = System.FAN_ALGORITHM_CONFIG[key][i+2]
                    start_idx = sensor_amount_range[0]
                    end_idx = sensor_amount_range[1]
                    while start_idx<=end_idx:
                        sensor_path = sensor_path_format % start_idx
                        s_params+=sensor_path + ";"
                        start_idx+=1
                    i+=3
                    continue
                s_params+=System.FAN_ALGORITHM_CONFIG[key][i] + ";"
                i+=1
        except:
            return ""
        return s_params

    # Get the FRU area names defined in ID_LOOKUP table given a fru_id.
    # If serval areas are defined for a fru_id, the areas are returned
    # together as a string with each area name separated with ','.
    # If no fru area defined in ID_LOOKUP, an empty string will be returned.
    @dbus.service.method(DBUS_NAME, in_signature='y', out_signature='s')
    def getFRUArea(self, fru_id):
        ret_str = ''
        fru_id = '_' + str(fru_id)
        area_list = [
            area for area in System.ID_LOOKUP['FRU_STR'].keys()
            if area.endswith(fru_id)]
        for area in area_list:
            ret_str = area + ',' + ret_str
        # remove the last ','
        return ret_str[:-1]

    def NewObjectHandler(self, obj_path, iprops, bus_name=None):
        current_state = self.Get(DBUS_NAME, "current_state")
        if current_state not in System.EXIT_STATE_DEPEND:
            return

        if obj_path in System.EXIT_STATE_DEPEND[current_state]:
            print "New object: "+obj_path+" ("+bus_name+")"

    @dbus.service.method(DBUS_NAME, in_signature='s', out_signature='sis')
    def gpioInit(self, name):
        gpio_path = ''
        gpio_num = -1
        r = ['', gpio_num, '']
        if name not in System.GPIO_CONFIG:
            # TODO: Better error handling
            msg = "ERROR: "+name+" not found in GPIO config table"
            print(msg)
            raise Exception(msg)
        else:

            gpio_num = -1
            gpio = System.GPIO_CONFIG[name]
            if 'gpio_num' in System.GPIO_CONFIG[name]:
                gpio_num = gpio['gpio_num']
            else:
                if 'gpio_pin' in System.GPIO_CONFIG[name]:
                    gpio_num = obmc.system.convertGpio(gpio['gpio_pin'])
                else:
                    msg = "ERROR: SystemManager - GPIO lookup failed for "+name
                    print(msg)
                    raise Exception(msg)

            if (gpio_num != -1):
                r = [obmc.enums.GPIO_DEV, gpio_num, gpio['direction']]
        return r

    @dbus.service.method(DBUS_NAME, in_signature='',
                         out_signature='ssa(sb)a(sb)a(sbb)ssssa(sb)')
    def getGpioConfiguration(self):
        power_config = System.GPIO_CONFIGS.get('power_config', {})
        power_good_in = power_config.get('power_good_in', '')
        latch_out = power_config.get('latch_out', '')
        power_up_outs = power_config.get('power_up_outs', [])
        reset_outs = power_config.get('reset_outs', [])
        pci_reset_outs = power_config.get('pci_reset_outs', [])
        hostctl_config = System.GPIO_CONFIGS.get('hostctl_config', {})
        fsi_data = hostctl_config.get('fsi_data', '')
        fsi_clk = hostctl_config.get('fsi_clk', '')
        fsi_enable = hostctl_config.get('fsi_enable', '')
        cronus_sel = hostctl_config.get('cronus_sel', '')
        optionals = hostctl_config.get('optionals', [])
        r = [power_good_in, latch_out, power_up_outs, reset_outs,
             pci_reset_outs, fsi_data, fsi_clk, fsi_enable, cronus_sel,
             optionals]
        print("Power GPIO config: " + str(r))
        return r


if __name__ == '__main__':
    dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
    bus = get_dbus()
    obj = SystemManager(bus, OBJ_NAME)
    mainloop = gobject.MainLoop()
    obj.unmask_signals()
    name = dbus.service.BusName(DBUS_NAME, bus)

    print("Running SystemManager")
    mainloop.run()

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
