import os

import spacetrack.operators as op
from pyorbital.orbital import Orbital
from requests import get
from spacetrack import SpaceTrackClient

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
    print(lines)
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
        # TODO check if int
        ids.append(fn.split(' ')[0])
    return ids


def get_all_names():
    names = []
    for fn in get_all_full_names():
        fn = fn.rstrip().split(' ')
        # TODO check if int
        fn.pop(0)
        names.append(' '.join(fn))
    return names


def get_tle_by_name(name):
    name_idx = get_all_names().index(name)
    lines = get_all_lines()
    # TODO exception
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


# TODO
# def update_tle(sat_id=None, upd_all=True):
#     if upd_all:
#         ids = get_all_ids()
#         for sat_id in ids:
#             TleHandler(sat_id)


class TleHandler:

    def __init__(self, sat_id=None, tle=None):
        # TODO refactor it!
        self.already_exists = None
        self.sat_id = None
        self.tle = None

        if sat_id is not None and tle is not None:
            # TODO error
            pass

        if (sat_id is not None and str(sat_id) in get_all_ids()) \
            or (tle is not None and str(tle[0].split(' ').pop(0)) in get_all_ids()):
            # TODO return satellite name to the user
            self.already_exists = True
        elif sat_id is not None or tle is not None:
            if sat_id is not None:
                # TODO make something with freeze
                self.sat_id = sat_id

                self.tle = self.__get_celestrak_tle()
                if self.tle is None:
                    self.tle = self.__get_spacetrack_tle()
            elif tle is not None:
                self.tle = tle

            if self.tle is None:
                # TODO raise and handle exception
                pass
            else:
                # TODO add TLE check
                self.orb = Orbital(satellite=self.tle[0], line1=self.tle[1], line2=self.tle[2])
                save_tle(self.tle)

    def __get_spacetrack_tle(self, start_date=None, end_date=None, username=ST_USERNAME, password=ST_PASSWORD,
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
            data = stc.tle(norad_cat_id=self.sat_id, orderby='epoch desc', limit=1, format='3le', epoch=date_range)
        else:
            data = stc.tle_latest(norad_cat_id=self.sat_id, orderby='epoch desc', limit=1, format='3le')

        if not data:
            return None
        else:
            tle = data.split('\n')
            name = tle[0].split(' ')
            name.pop(0)
            tle[0] = str(self.sat_id) + ' ' + str(' '.join(name))
            return tle

    def __get_celestrak_tle(self):
        tle = get("http://www.celestrak.com/satcat/tle.php?CATNR={}".format(self.sat_id)).text.strip().split("\r\n")
        if tle == ["No TLE found"]:
            tle = None
        else:
            tle[0] = str(self.sat_id) + ' ' + str(tle[0])
        return tle
        # satellite = ephem.readtle(*tle)
