#!/usr/bin/env python3
"""Run on an expanded URDF: python3 scripts/check_exercise.py /tmp/mas248_arm.urdf

Uses Python standard library only. Checks the intended student exercise goals.
"""
import math
import sys
import xml.etree.ElementTree as ET


def main(path):
    root = ET.parse(path).getroot()
    links = {link.attrib['name'] for link in root.findall('link')}
    joints = {joint.attrib['name']: joint for joint in root.findall('joint')}
    assert {'base_link', 'link1', 'link2', 'tcp'} <= links, 'Missing basic arm links'
    assert {'joint1', 'joint2', 'tcp_joint'} <= set(joints), 'Missing basic joints'
    j2 = joints['joint2']
    assert j2.attrib['type'] == 'revolute'
    limit = j2.find('limit')
    low, high = float(limit.get('lower')), float(limit.get('upper'))
    print('joint2 limits:', low, 'to', high, 'radians')
    if math.isclose(low, -0.8, abs_tol=0.001) and math.isclose(high, 0.8, abs_tol=0.001):
        print('[PASS] Exercise 1: narrowed joint2 limits to +/-0.80 rad')
    else:
        print('[TODO] Exercise 1: expected joint2 limits -0.80 to +0.80 rad')

    fixed = joints.get('fixture_joint')
    if ('fixture_link' in links and fixed is not None and
        fixed.attrib.get('type') == 'fixed' and
        fixed.find('parent') is not None and
        fixed.find('child') is not None and
        fixed.find('parent').get('link') == 'base_link' and
        fixed.find('child').get('link') == 'fixture_link'):
        print('[PASS] Exercise 2: fixed fixture_link attached to base_link')
    else:
        print('[TODO] Exercise 2: add fixture_link and fixed fixture_joint to base_link')

    tcp = joints['tcp_joint']
    assert tcp.attrib['type'] == 'fixed' and tcp.find('child').get('link') == 'tcp'
    print('[PASS] TCP fixed joint preserved')


if __name__ == '__main__':
    if len(sys.argv) != 2:
        sys.exit('Usage: python3 scripts/check_exercise.py /tmp/mas248_arm.urdf')
    main(sys.argv[1])
