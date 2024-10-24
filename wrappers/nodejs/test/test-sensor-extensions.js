// Copyright (c) 2017 Intel Corporation. All rights reserved.
// Use of this source code is governed by an Apache 2.0 license
// that can be found in the LICENSE file.

'use strict';
/* global describe, it, before, after */
const assert = require('assert');
let rs2;
// let pipeline;
try {
  rs2 = require('node-librealsense');
} catch (e) {
  rs2 = require('../index.js');
}

let device;
// let frameset;
let devInfo;
let motionProductList = ['0AD5', '0AFE', '0AFF', '0B00', '0B01', '0B3A', '0B3D'];
const ctx = new rs2.Context();
let devices = ctx.queryDevices().devices;
assert(devices.length > 0); // Device must be connected
device = devices[0];
devInfo = device.getCameraInfo();


describe('Sensor extensions test', function() {
  before(function() {
    // const ctx = new rs2.Context();
    // let devices = ctx.queryDevices().devices;
    // assert(devices.length > 0); // Device must be connected
    // device = devices[0]
    // devInfo = device.getCameraInfo();
  });

  after(function() {
    rs2.cleanup();
  });

  it('Testing constructor - 0 argument', () => {
    assert.throws(() => {
      new rs2.Align();
    });
  });

  it('Testing ColorSensor extention', () => {
      const productsList = ['0AA5, 0B48, 0AD3, 0AD4, 0AD5, 0B01, 0B07, 0B3A, 0B3D'];
      let rtn = device.getCameraInfo();
      if (productsList.includes(rtn.productId)) {
        const sensors = device.querySensors();
        let isFound = false;
        for (let i = 0; i < sensors.length; i++) {
          if (sensors[i] instanceof rs2.ColorSensor) {
            const sensor = sensors[i];
            const profile = sensor.getStreamProfiles()[0];
            assert(profile.streamType === rs2.stream.STREAM_COLOR &&
              profile.format == rs2.format.FORMAT_RGB8);
            isFound = true;
          }
        }
        assert(isFound);
      }
  });

  it('Testing FisheyeSensor extention', () => {
    const productsList = ['0AD5', '0AFE', '0AFF', '0B00', '0B01'];
    let rtn = device.getCameraInfo();
    if (productsList.includes(rtn.productId)) {
      const sensors = device.querySensors();
      let isFound = false;
      for (let i = 0; i < sensors.length; i++) {
        if (sensors[i] instanceof rs2.FisheyeSensor) {
          const sensor = sensors[i];
          const profile = sensor.getStreamProfiles()[0];
          assert(profile.streamType === rs2.stream.STREAM_COLOR &&
            profile.format == rs2.format.FORMAT_RGB8);
          isFound = true;
        }
      }
      assert(isFound);
    }
  });

  if (motionProductList.includes(devInfo.productId)) {
    it('Testing MotionSensor extention', () => {
      const productsList = ['0AD5', '0AFE', '0AFF', '0B00', '0B01', '0B3A', '0B3D'];
      let rtn = device.getCameraInfo();
      if (productsList.includes(rtn.productId)) {
        const sensors = device.querySensors();
        let isFound = false;
        for (let i = 0; i < sensors.length; i++) {
          if (sensors[i] instanceof rs2.MotionSensor) {
            const sensor = sensors[i];
            const profile = sensor.getStreamProfiles()[0];
            assert([rs2.stream.STREAM_GYRO, rs2.stream.STREAM_ACCEL].includes(profile.streamType));
            assert([rs2.format.FORMAT_MOTION_RAW, rs2.format.FORMAT_MOTION_XYZ32F]
              .includes(profile.format));
            isFound = true;
          }
        }
        assert(isFound);
      }
    });
  }
});
