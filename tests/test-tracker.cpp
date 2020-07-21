/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file test-tracker.cpp
 * @authors: Ugo Pattacini <ugo.pattacini@iit.it>
 */

#include <cstdlib>
#include <memory>
#include <string>
#include <sstream>
#include <limits>
#include <yarp/os/Network.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Vocab.h>
#include <yarp/os/Value.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/LogStream.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>
#include "AssistiveRehab/skeleton.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace assistive_rehab;

class Tracker : public RFModule {
  BufferedPort<Bottle> opc;
  BufferedPort<Property> gaze_rx;
  BufferedPort<Property> gaze_tx;
  Matrix gaze_frame;
  string keypoint;
  bool use_pixel;

  bool configure(ResourceFinder& rf)override {
    use_pixel = rf.check("use-pixel", Value(true)).asBool();
    opc.open("/test-tracker/skeletons:i");
    if (!Network::connect("/opc/broadcast:o", opc.getName())) {
      yError() << "Unable to connect to OPC!";
      opc.close();
      return false;
    }
    gaze_rx.open("/test-tracker/gaze/rx");
    gaze_tx.open("/test-tracker/gaze/tx");
    if (!Network::connect("/cer_gaze-controller/state:o", gaze_rx.getName()) ||
        !Network::connect(gaze_tx.getName(), "/cer_gaze-controller/target:i")) {
      yError() << "Unable to connect to Gaze Controller!";
      opc.close();
      gaze_rx.close();
      gaze_tx.close();
      return false;
    }
    return true;
  }

  double getPeriod()override {
    return 0.0;
  }

  bool updateModule()override {
    if (Bottle* skeletons = opc.read()) {
      string tag = skeletons->get(0).asString();
      if ((tag == "sync") || (tag == "async")) {
        Vector fp, pixel;
        ostringstream skeletons_info;
        double min_d = numeric_limits<double>::infinity();
        for (size_t i = 1; i < skeletons->size(); i++) {
          Value v = skeletons->get(i);
          if (v.isList()) {
            Property prop(v.asList()->toString().c_str());
            unique_ptr<Skeleton> sk(skeleton_factory(prop));
            if ((*sk)[keypoint]->isUpdated()) {
              Vector x = (*sk)[keypoint]->getPoint();
              Vector px = (*sk)[keypoint]->getPixel();
              double d = norm(x);
              skeletons_info << sk->getTag() << ": ("
                  << x.toString(3, 3) << "), ("
                  << px.toString(1, 1) << "); ";
              if (d < min_d) {
                fp = x;
                pixel = px;
                min_d = d;
              }
            }
          }
        }
        if (fp.size() > 0) {
          if (Property* prop = gaze_rx.read(false)) {
            Vector pose;
            prop->find("depth_rgb").asList()->write(pose);
            gaze_frame = axis2dcm(pose.subVector(3, 6));
            gaze_frame.setSubcol(pose.subVector(0, 2), 0, 3);
          }

          Bottle loc;
          Property& options = gaze_tx.prepare();
          options.put("control-frame", "depth_rgb");
          if (use_pixel) {
            loc.addList().read(pixel);
            options.put("target-type", "image");
            options.put("image", "depth_rgb");
          } else {
            fp.push_back(1.0);
            fp = gaze_frame * fp;
            fp.pop_back();
            loc.addList().read(fp);
            options.put("target-type", "cartesian");
          }
          options.put("target-location", loc.get(0));
          yInfo() << "skeletons info: " << skeletons_info.str();
          yInfo() << "gazing at:" << options.toString();
          gaze_tx.writeStrict();
        }
      }
    }
    return true;
  }

  bool interruptModule()override {
    opc.interrupt();
    gaze_rx.interrupt();
    return true;
  }

  bool close()override {
    Bottle loc;
    loc.addList().read(zeros(2));
    Property& options = gaze_tx.prepare();
    options.put("control-frame", "depth_rgb");
    options.put("target-type", "angular");
    options.put("target-location", loc.get(0));
    gaze_tx.writeStrict();

    opc.close();
    gaze_rx.close();
    gaze_tx.close();
    return true;
  }

public:

  Tracker() : keypoint(KeyPointTag::shoulder_center) { }
};

int main(int argc, char* argv[]) {
  Network yarp;
  if (!yarp.checkNetwork()) {
    yError() << "Unable to talk to YARP server!";
    return EXIT_FAILURE;
  }

  ResourceFinder rf;
  rf.configure(argc, argv);

  Tracker tracker;
  return tracker.runModule(rf);
}
