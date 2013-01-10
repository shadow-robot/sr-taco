/*
 * attention_manager.h
 *
 *  Created on: 14 Dec 2012
 *      Author: mda
 */

#pragma once

#include <nodelet/nodelet.h>
#include "sr_taco_openni/common.h"

namespace sr_taco_openni
{
  /**
   * \brief Base class for attention managers, nodelets that generate saliency maps.
   */
  class AttentionManager : public nodelet::Nodelet
  {
    public:
      AttentionManager(){}
  };
}
