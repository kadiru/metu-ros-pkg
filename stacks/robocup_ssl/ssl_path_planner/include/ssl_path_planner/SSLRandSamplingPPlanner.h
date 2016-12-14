/*
 * SSLRandSamplingPPlanner.h
 *
 *  Created on: Dec 21, 2011
 *      Author: kadir
 */

#ifndef SSLRANDSAMPLINGPPLANNER_H_
#define SSLRANDSAMPLINGPPLANNER_H_

#include "SSLPathPlanner.h"

namespace ssl {

class SSLRandSamplingPPlanner: public ssl::SSLPathPlanner {
public:
	SSLRandSamplingPPlanner(ros::NodeHandle* nh);
	virtual ~SSLRandSamplingPPlanner();
};

}

#endif /* SSLRANDSAMPLINGPPLANNER_H_ */
