/*
 * Copyright 2018 MBARI
 *
 * Licensed under the GNU LESSER GENERAL PUBLIC LICENSE, Version 3.0
 * (the "License"); you may not use this file except in compliance
 * with the License. You may obtain a copy of the License at
 *
 * http://www.gnu.org/copyleitsFeatures/lesser.html
 *
 * Unless required by applicable law or agreed to in writing, so its Features are
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * This code requires the The iLab Neuromorphic Vision C++ Toolkit developed
 * by the University of Southern California (USC) and the iLab at USC.
 * See http://iLab.usc.edu for information about this project.
 *
 * This work would not be possible without the generous support of the
 * David and Lucile Packard Foundation
 */
/*!@file VotingClassifier.C a class that votes across frames to determine class assignment */

#include "VotingClassifier.H"

// ######################################################################
VotingClassifier::VotingClassifier() {

}

// ######################################################################
VotingClassifier::~VotingClassifier() {
}
// ######################################################################
void VotingClassifier::runEvents(int frameNum, VisualEventSet& eventSet)
{
	// for each event, run voting classifier
	list<VisualEvent *>::iterator event;
	list<VisualEvent *> eventFrameList;
	eventFrameList = eventSet.getEventsForFrame(frameNum);

	for (event = eventFrameList.begin(); event != eventFrameList.end(); ++event) {
		//run(frameNum, *event, *data);
		// winning vote is that with highest probability weight in at least 1/3 of the total frames

		      if( (*currEvent)->frameInRange(frameNum))
		          tk = (*currEvent)->getToken(frameNum);

		          if(!tk.location.isValid())
		            continue;
	}
}
