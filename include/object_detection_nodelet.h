#ifndef OBJECT_DETECTION_NODELET_H_
#define OBJECT_DETECTION_NODELET_H_

#include <nodelet/nodelet.h>
#include <object_detection.h>

namespace point_cloud {

class ObjectDetectionNodelet: public nodelet::Nodelet {
  public:
    virtual void onInit();
  private:
    ObjectDetection* sj;
};

}

#endif /* OBJECT_DETECTION_NODELET_H_ */
