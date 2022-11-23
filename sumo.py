#!/usr/bin/python

import lxml.etree
import pandas
import numpy as np
import os.path
import logging
import json
import sys

class SUMOReader(object):
    def __init__(self, sumocfg):
        # Parse sumo configuration for basic information
        x = lxml.etree.XML(open(sumocfg, 'rb').read())

        dirname = os.path.dirname(sumocfg)

        output_prefix = os.path.join(dirname, x.xpath('//output/output-prefix/@value')[0])
        
        self._net_filename = os.path.join(dirname, x.xpath('//input/net-file/@value')[0])

        self._summary_filename  = output_prefix + x.xpath('//output/summary-output/@value')[0]
        self._tripinfo_filename = output_prefix + x.xpath('//output/tripinfo-output/@value')[0]
        self._vehroute_filename = output_prefix + x.xpath('//output/vehroute-output/@value')[0]
        self._fcd_filename      = output_prefix + x.xpath('//output/fcd-output/@value')[0]

        # Random seed used by SUMO (or None if not available
        self.seed = (x.xpath('//random_number/seed/@value') + [None])[0]

        self.timing = (
            float(x.xpath('//time/begin/@value')[0]),
            float(x.xpath('//time/end/@value')[0]),
            float(x.xpath('//time/step-length/@value')[0]),
        )

        self.tick = -1

        # Import junctions and vehicle ids statically (fit in memory)
        logging.info("Import junctions...")
        self.junctions = self.parse_junctions()
        logging.info("%d junctions imported." % len(self.junctions))

        logging.info("Import vehicle ids")
        self.vehicles = self.parse_vehicles()
        logging.info("%d vehicles imported." % len(self.vehicles))

    def __iter__(self):
        self._fcd_iter = lxml.etree.iterparse(open(self._fcd_filename, 'rb'))
        return self

    def __next__(self):
        ret = []
        while True:
            _, elem = next(self._fcd_iter)

            if elem.tag == 'timestep':
                self.tick = float(elem.attrib['time'])
                elem.clear()
                break
            elif elem.tag == 'fcd-export':
                raise StopIteration
            elif elem.tag == 'vehicle':
                ret.append((elem.attrib['id'], (
                    float(elem.attrib['x']),
                    float(elem.attrib['y']),
                    float(elem.attrib['angle']),
                )))
            elif elem.tag == 'person':
                pass
            else:
                raise AssertionError("Unknown tag %s" % elem.tag)

        return ret

    def parse_vehicles(self):
        x = lxml.etree.XML(open(self._tripinfo_filename, 'rb').read())

        ret = []
        for ti in x.xpath('//tripinfo'):
            ti = ti.attrib
            ret.append((ti['id'], {
                'type': ti['vType'],
                'depart': float(ti['depart']),
                'arrival': float(ti['arrival']),
            }))

        return dict(ret)

    def parse_junctions(self, min_lanes=10) -> pandas.DataFrame:
        x = lxml.etree.XML(open(self._net_filename, 'rb').read())

        ret = []
        for j in x.xpath('//junction'):
            j = j.attrib
            if len(j['intLanes'].split(' ')) < min_lanes:
                continue

            ret.append((j['id'],(float(j['x']), float(j['y']))))

        return dict(ret)

if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG)

    if len(sys.argv) != 3:
        print("%s /path/to/filename.sumocfg /path/to/output/" % sys.argv[0])
        exit(-1)

    assert sys.argv[1].endswith('.sumocfg')

    sr = SUMOReader(sys.argv[1])

    os.makedirs(sys.argv[2], exist_ok=True)

    for i in sr:
        open(os.path.join(sys.argv[2], "%d.json" % sr.tick), 'w').write(json.dumps(i))
