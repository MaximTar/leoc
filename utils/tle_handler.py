import os

import spacetrack.operators as op
from enum import Enum
from pyorbital.orbital import Orbital
from requests import get
from spacetrack import SpaceTrackClient

# Work with indexes and satellite identifiers is preferable, since names can be duplicated

ST_USERNAME = "max_131092@mail.ru"
ST_PASSWORD = "Acslacslacslacsl"

dir_name = os.path.dirname(os.path.realpath(__file__))
file_name = os.path.join(dir_name, '../resources/tle')
if not os.path.exists(file_name):
    with open(file_name, 'w'):
        pass


def save_tle(tle):
    if not already_in_file(tle):
        f = open(file_name, 'a')
        for t in tle:
            f.write(t + '\n')
        f.close()


def remove_tle_by_name(name):
    name_idx = get_all_names().index(name)
    lines = get_all_lines()
    del lines[name_idx * 3:name_idx * 3 + 3]
    f = open(file_name, 'w')
    for line in lines:
        f.write(line)
    f.close()


def remove_tle_by_index(index):
    lines = get_all_lines()
    del lines[index * 3:index * 3 + 3]
    f = open(file_name, 'w')
    for line in lines:
        f.write(line)
    f.close()


def already_in_file(tle):
    with open(file_name) as f:
        line = f.readline()
        line_number = 0
        while line:
            if line_number % 3 == 0 and tle[0].rstrip() == line.rstrip():
                return True
            line_number += 1
            line = f.readline()
    return False


def get_all_lines():
    with open(file_name) as f:
        lines = f.readlines()
    return lines


def get_all_full_names():
    return get_all_lines()[0::3]


def get_all_ids():
    ids = []
    for fn in get_all_full_names():
        sat_id = fn.split(' ')[0]
        if is_sat_id(sat_id):
            ids.append(fn.split(' ')[0])
    return ids


def get_all_names():
    names = []
    for fn in get_all_full_names():
        fn = fn.rstrip().split(' ')
        if is_sat_id(fn[0]):
            fn.pop(0)
        names.append(' '.join(fn))
    return names


def get_tle_by_name(name):
    name_idx = get_all_names().index(name)
    lines = get_all_lines()
    return lines[name_idx * 3 + 1], lines[name_idx * 3 + 2]


def get_tle_list_by_names(names):
    tle_list = []
    for name in names:
        tle_list.append(get_tle_by_name(name))
    return tle_list


def get_orb_list_by_tle_list(tle_list):
    orb_list = []
    for tle in tle_list:
        if len(tle) == 2:
            orb = Orbital(satellite="", line1=tle[0], line2=tle[1])
            orb_list.append(orb)
        elif len(tle) == 3:
            orb = Orbital(satellite=tle[0], line1=tle[1], line2=tle[2])
            orb_list.append(orb)
    return orb_list


def get_sat_id_by_name(name):
    return get_all_ids()[get_all_names().index(name)]


def get_name_by_sat_id(sat_id):
    return get_all_names()[get_all_ids().index(sat_id)]


# preferred method
def update_tle_by_index(index):
    lines = get_all_lines()
    if is_sat_id(lines[index * 3].split(' ')[0]):
        sat_id = lines[index * 3].split(' ').pop(0)
        update_tle_by_sat_id(sat_id)
    else:
        # TODO message?
        pass


def update_tle_by_name(name):
    update_tle_by_sat_id(get_sat_id_by_name(name))


def update_tle_by_sat_id(sat_id):
    tle = get_tle_by_sat_id(sat_id)
    if tle is None:
        # TODO message?
        pass
    else:
        tle = ['{}\n'.format(e) for e in tle]
        lines = get_all_lines()
        idx = get_all_ids().index(sat_id)
        lines[idx * 3:idx * 3 + 3] = tle
        with open(file_name, 'w') as file:
            file.writelines(lines)


def update_all_tles():
    ids = get_all_ids()
    for sat_id in ids:
        # TODO write another method (now this rewrite file for every TLE)
        update_tle_by_sat_id(sat_id)


def _get_spacetrack_tle(sat_id, start_date=None, end_date=None, username=ST_USERNAME, password=ST_PASSWORD,
                        latest=True):
    """
    For an satellite with the given id returns the TLE.
    Also needs date range and spacetrack username and password.
    Optional flag is used to get the latest data.
    :param start_date:
    :param end_date:
    :param username:
    :param password:
    :param latest:
    :return: two TLE string arrays
    """
    stc = SpaceTrackClient(identity=username, password=password)
    if not latest:
        date_range = op.inclusive_range(start_date, end_date)
        data = stc.tle(norad_cat_id=sat_id, orderby='epoch desc', limit=1, format='3le', epoch=date_range)
    else:
        data = stc.tle_latest(norad_cat_id=sat_id, orderby='epoch desc', limit=1, format='3le')

    if not data:
        return None
    else:
        tle = data.split('\n')
        name = tle[0].split(' ')
        name.pop(0)
        tle[0] = str(sat_id) + ' ' + str(' '.join(name))
        # sometimes there may be empty elements
        return tle[0], tle[1], tle[2]


def _get_celestrak_tle(sat_id):
    tle = get("http://www.celestrak.com/satcat/tle.php?CATNR={}".format(sat_id)).text.strip().split("\r\n")
    if tle == ["No TLE found"]:
        tle = None
    else:
        tle[0] = str(sat_id) + ' ' + str(tle[0])
    return tle
    # satellite = ephem.readtle(*tle)


def get_tle_by_sat_id(sat_id):
    if sat_id is None or not is_sat_id(sat_id):
        return None
    tle = _get_celestrak_tle(sat_id)
    if tle is None:
        tle = _get_spacetrack_tle(sat_id)
    return tle


def is_sat_id(sat_id):
    # noinspection PyBroadException
    try:
        sat_id = int(sat_id)
    except BaseException as e:
        return False
    return sat_id > 0


class TleHandler:
    class Result(Enum):
        IS_NONE = 1
        ALREADY_EXISTS = 2
        SAVED = 3

    def save_by_tle(self, tle):
        if tle is None:
            return self.Result.IS_NONE
        else:
            sat_id = tle[0].split(' ')[0]
        if is_sat_id(sat_id):
            if tle is not None and str(sat_id) in get_all_ids():
                return self.Result.ALREADY_EXISTS
            elif tle is not None:
                tle[0].split(' ').pop(0)
                # TODO add TLE check
                save_tle(tle)
                return self.Result.SAVED
        else:
            return self.Result.IS_NONE

    def save_by_sat_id(self, sat_id):
        if sat_id is None or not is_sat_id(sat_id):
            return self.Result.IS_NONE
        elif sat_id is not None and str(sat_id) in get_all_ids():
            return self.Result.ALREADY_EXISTS
        elif sat_id is not None:
            # TODO make something with freeze (thread?)
            tle = _get_celestrak_tle(sat_id)
            if tle is None:
                tle = _get_spacetrack_tle(sat_id)
            self.save_by_tle(tle)
            return self.Result.SAVED
