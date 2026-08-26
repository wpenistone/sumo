/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.dev/sumo
// Copyright (C) 2001-2026 German Aerospace Center (DLR) and others.
// This program and the accompanying materials are made available under the
// terms of the Eclipse Public License 2.0 which is available at
// https://www.eclipse.org/legal/epl-2.0/
// This Source Code may also be made available under the following Secondary
// Licenses when the conditions for such availability set forth in the Eclipse
// Public License 2.0 are satisfied: GNU General Public License, version 2
// or later which is available at
// https://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
// SPDX-License-Identifier: EPL-2.0 OR GPL-2.0-or-later
/****************************************************************************/
/// @file    NIImporter_OpenStreetMap.cpp
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @author  Walter Bamberger
/// @author  Gregor Laemmel
/// @author  Mirko Barthauer
/// @author  William Harrison Davis
/// @date    Mon, 14.04.2008
///
// Importer for networks stored in OpenStreetMap format
/****************************************************************************/
#include <config.h>
#include <algorithm>
#include <set>
#include <functional>
#include <sstream>
#include <limits>
#include <utils/common/UtilExceptions.h>
#include <utils/common/StringUtils.h>
#include <utils/common/ToString.h>
#include <utils/common/MsgHandler.h>
#include <utils/common/StringUtils.h>
#include <utils/common/StringTokenizer.h>
#include <utils/common/FileHelpers.h>
#include <utils/geom/GeoConvHelper.h>
#include <utils/geom/GeomConvHelper.h>
#include <utils/options/OptionsCont.h>
#include <utils/xml/SUMOSAXHandler.h>
#include <utils/xml/SUMOSAXReader.h>
#include <utils/xml/SUMOXMLDefinitions.h>
#include <utils/xml/XMLSubSys.h>
#include <netbuild/NBEdge.h>
#include <netbuild/NBEdgeCont.h>
#include <netbuild/NBNode.h>
#include <netbuild/NBNodeCont.h>
#include <netbuild/NBNetBuilder.h>
#include "NIOSMCanonicalValues.h"
#include <netbuild/NBOwnTLDef.h>
#include <netbuild/NBPTLine.h>
#include <netbuild/NBPTLineCont.h>
#include <netbuild/NBPTPlatform.h>
#include <netbuild/NBPTStop.h>
#include "NILoader.h"
#include "NIImporter_OpenStreetMap.h"

//#define DEBUG_LAYER_ELEVATION
//#define DEBUG_RAIL_DIRECTION

// ---------------------------------------------------------------------------
// static members
// ---------------------------------------------------------------------------
const double NIImporter_OpenStreetMap::MAXSPEED_UNGIVEN = -1;

const long long int NIImporter_OpenStreetMap::INVALID_ID = std::numeric_limits<long long int>::max();
bool NIImporter_OpenStreetMap::myAllAttributes(false);
std::set<std::string> NIImporter_OpenStreetMap::myExtraAttributes;

// ===========================================================================
// Private classes
// ===========================================================================

/** @brief Functor which compares two Edges
 */
class NIImporter_OpenStreetMap::CompareEdges {
public:
    bool operator()(const Edge* e1, const Edge* e2) const {
        if (e1->myHighWayType != e2->myHighWayType) {
            return e1->myHighWayType > e2->myHighWayType;
        }
        if (e1->myNoLanes != e2->myNoLanes) {
            return e1->myNoLanes > e2->myNoLanes;
        }
        if (e1->myNoLanesForwardExplicit != e2->myNoLanesForwardExplicit) {
            return e1->myNoLanesForwardExplicit > e2->myNoLanesForwardExplicit;
        }
        if (e1->myNoLanesBackwardExplicit != e2->myNoLanesBackwardExplicit) {
            return e1->myNoLanesBackwardExplicit > e2->myNoLanesBackwardExplicit;
        }
        if (e1->myMaxSpeed != e2->myMaxSpeed) {
            return e1->myMaxSpeed > e2->myMaxSpeed;
        }
        if (e1->myIsOneWay != e2->myIsOneWay) {
            return e1->myIsOneWay > e2->myIsOneWay;
        }
        if (e1->myPlacement != e2->myPlacement) {
            return (int)e1->myPlacement > (int)e2->myPlacement;
        }
        if (e1->myPlacementLane != e2->myPlacementLane) {
            return e1->myPlacementLane > e2->myPlacementLane;
        }
        return e1->myCurrentNodes > e2->myCurrentNodes;
    }
};

// ===========================================================================
// method definitions
// ===========================================================================
// ---------------------------------------------------------------------------
// static methods
// ---------------------------------------------------------------------------
const std::string NIImporter_OpenStreetMap::compoundTypeSeparator("|"); //clang-tidy says: "compundTypeSeparator with
// static storage duration my throw an exception that cannot be caught

void
NIImporter_OpenStreetMap::loadNetwork(const OptionsCont& oc, NBNetBuilder& nb) {
    NIImporter_OpenStreetMap importer;
    importer.load(oc, nb);
}

NIImporter_OpenStreetMap::NIImporter_OpenStreetMap() = default;

NIImporter_OpenStreetMap::~NIImporter_OpenStreetMap() {
    // delete nodes
    for (auto myUniqueNode : myUniqueNodes) {
        delete myUniqueNode;
    }
    // delete edges
    for (auto& myEdge : myEdges) {
        delete myEdge.second;
    }
    // delete platform shapes
    for (auto& myPlatformShape : myPlatformShapes) {
        delete myPlatformShape.second;
    }
}

void
NIImporter_OpenStreetMap::load(const OptionsCont& oc, NBNetBuilder& nb) {
    if (!oc.isSet("osm-files")) {
        return;
    }
    const std::vector<std::string> files = oc.getStringVector("osm-files");
    std::vector<SUMOSAXReader*> readers;

    myImportLaneAccess = oc.getBool("osm.lane-access");
    myImportTurnSigns = oc.getBool("osm.turn-lanes");
    myImportSidewalks = oc.getBool("osm.sidewalks");
    myImportBikeAccess = oc.getBool("osm.bike-access");
    myImportCrossings = oc.getBool("osm.crossings");
    myOnewayDualSidewalk = oc.getBool("osm.oneway-reverse-sidewalk");
    myAnnotateDefaults = oc.getBool("osm.annotate-defaults");
    myPlacementSkippedNonExplicitOneWay = 0;
    myPlacementSkippedAuxOppositeDirection = 0;

    myAllAttributes = OptionsCont::getOptions().getBool("osm.all-attributes");
    std::vector<std::string> extra = OptionsCont::getOptions().getStringVector("osm.extra-attributes");
    myExtraAttributes.insert(extra.begin(), extra.end());
    if (myExtraAttributes.count("all") != 0) {
        // import all
        myExtraAttributes.clear();
    }

    // load nodes, first
    NodesHandler nodesHandler(myOSMNodes, myUniqueNodes, oc);
    for (const std::string& file : files) {
        if (!FileHelpers::isReadable(file)) {
            WRITE_ERRORF(TL("Could not open osm-file '%'."), file);
            return;
        }
        nodesHandler.setFileName(file);
        nodesHandler.resetHierarchy();
        const long before = PROGRESS_BEGIN_TIME_MESSAGE("Parsing nodes from osm-file '" + file + "'");
        readers.push_back(XMLSubSys::getSAXReader(nodesHandler));
        if (!readers.back()->parseFirst(file) || !readers.back()->parseSection(SUMO_TAG_NODE) ||
                MsgHandler::getErrorInstance()->wasInformed()) {
            return;
        }
        if (nodesHandler.getDuplicateNodes() > 0) {
            WRITE_MESSAGEF(TL("Found and substituted % osm nodes."), toString(nodesHandler.getDuplicateNodes()));
        }
        PROGRESS_TIME_MESSAGE(before);
    }

    // load edges, then
    EdgesHandler edgesHandler(myOSMNodes, myEdges, myPlatformShapes, nb.getTypeCont());
    int idx = 0;
    for (const std::string& file : files) {
        edgesHandler.setFileName(file);
        readers[idx]->setHandler(edgesHandler);
        const long before = PROGRESS_BEGIN_TIME_MESSAGE("Parsing edges from osm-file '" + file + "'");
        if (!readers[idx]->parseSection(SUMO_TAG_WAY)) {
            // eof already reached, no relations
            delete readers[idx];
            readers[idx] = nullptr;
        }
        PROGRESS_TIME_MESSAGE(before);
        idx++;
    }

    /* Remove duplicate edges with the same shape and attributes */
    if (!oc.getBool("osm.skip-duplicates-check")) {
        int numRemoved = 0;
        PROGRESS_BEGIN_MESSAGE(TL("Removing duplicate edges"));
        if (myEdges.size() > 1) {
            std::set<const Edge*, CompareEdges> dupsFinder;
            for (auto it = myEdges.begin(); it != myEdges.end();) {
                if (dupsFinder.count(it->second) > 0) {
                    numRemoved++;
                    delete it->second;
                    myEdges.erase(it++);
                } else {
                    dupsFinder.insert(it->second);
                    it++;
                }
            }
        }
        if (numRemoved > 0) {
            WRITE_MESSAGEF(TL("Removed % duplicate osm edges."), toString(numRemoved));
        }
        PROGRESS_DONE_MESSAGE();
    }

    /* Mark which nodes are used (by edges or traffic lights).
     * This is necessary to detect which OpenStreetMap nodes are for
     * geometry only */
    std::map<long long int, int> nodeUsage;
    // Mark which nodes are used by edges (begin and end)
    for (const auto& edgeIt : myEdges) {
        assert(edgeIt.second->myCurrentIsRoad);
        for (const long long int node : edgeIt.second->myCurrentNodes) {
            nodeUsage[node]++;
        }
    }
    // Mark which nodes are used by traffic lights or are pedestrian crossings
    for (const auto& nodesIt : myOSMNodes) {
        if (nodesIt.second->tlsControlled || nodesIt.second->railwaySignal || (nodesIt.second->pedestrianCrossing && myImportCrossings) /* || nodesIt->second->railwayCrossing*/) {
            // If the key is not found in the map, the value is automatically
            // initialized with 0.
            nodeUsage[nodesIt.first]++;
        }
    }

    /* Instantiate edges
     * Only those nodes in the middle of an edge which are used by more than
     * one edge are instantiated. Other nodes are considered as geometry nodes. */
    NBNodeCont& nc = nb.getNodeCont();
    NBTrafficLightLogicCont& tlsc = nb.getTLLogicCont();
    for (const auto& edgeIt : myEdges) {
        Edge* const e = edgeIt.second;
        if (!e->myCurrentIsRoad) {
            continue;
        }
        if (e->myCurrentNodes.size() < 2) {
            WRITE_WARNINGF(TL("Discarding way '%' because it has only % node(s)"), e->id, e->myCurrentNodes.size());
            continue;
        }
        extendRailwayDistances(e, nb.getTypeCont());
        // build nodes;
        //  - the from- and to-nodes must be built in any case
        //  - the in-between nodes are only built if more than one edge references them
        NBNode* first = insertNodeChecking(e->myCurrentNodes.front(), nc, tlsc);
        NBNode* last = insertNodeChecking(e->myCurrentNodes.back(), nc, tlsc);
        NBNode* currentFrom = first;
        int running = 0;
        std::vector<long long int> passed;
        for (auto j = e->myCurrentNodes.begin(); j != e->myCurrentNodes.end(); ++j) {
            passed.push_back(*j);
            if (nodeUsage[*j] > 1 && j != e->myCurrentNodes.end() - 1 && j != e->myCurrentNodes.begin()) {
                NBNode* currentTo = insertNodeChecking(*j, nc, tlsc);
                running = insertEdge(e, running, currentFrom, currentTo, passed, nb, first, last);
                currentFrom = currentTo;
                passed.clear();
                passed.push_back(*j);
            }
        }
        if (running == 0) {
            running = -1;
        }
        insertEdge(e, running, currentFrom, last, passed, nb, first, last);
    }
    if (myPlacementSkippedNonExplicitOneWay > 0 || myPlacementSkippedAuxOppositeDirection > 0) {
        WRITE_MESSAGEF(TL("Skipped applying OSM placement on % edge(s): % due to non-explicit one-way and % due to opposite-direction auxiliary edges."),
                       myPlacementSkippedNonExplicitOneWay + myPlacementSkippedAuxOppositeDirection,
                       myPlacementSkippedNonExplicitOneWay,
                       myPlacementSkippedAuxOppositeDirection);
    }

    /* Collect edges which explicitly are part of a roundabout and store the edges of each
     * detected roundabout */
    nb.getEdgeCont().extractRoundabouts();

    if (myImportCrossings) {
        /* After edges are instantiated
         * nodes are parsed again to add pedestrian crossings to them
         * This is only executed if crossings are imported and not guessed */
        const double crossingWidth = OptionsCont::getOptions().getFloat("default.crossing-width");

        for (auto item : nodeUsage) {
            NIOSMNode* osmNode = myOSMNodes.find(item.first)->second;
            if (osmNode->pedestrianCrossing) {
                NBNode* n = osmNode->node;
                EdgeVector incomingEdges = n->getIncomingEdges();
                EdgeVector outgoingEdges = n->getOutgoingEdges();
                size_t incomingEdgesNo = incomingEdges.size();
                size_t outgoingEdgesNo = outgoingEdges.size();

                for (size_t i = 0; i < incomingEdgesNo; i++) {
                    /* Check if incoming edge has driving lanes(and sidewalks)
                     * if not, ignore
                     * if yes, check if there is a corresponding outgoing edge for the opposite direction
                     *   -> if yes, check if it has driving lanes
                     *          --> if yes, do the crossing
                     *          --> if no, only do the crossing with the incoming edge (usually one lane roads with two sidewalks)
                     *   -> if not, do nothing as we don't have a sidewalk in the opposite direction */
                    auto const iEdge = incomingEdges[i];

                    if (iEdge->getFirstNonPedestrianLaneIndex(NBNode::FORWARD) > -1
                            && iEdge->getSpecialLane(SVC_PEDESTRIAN) > -1) {
                        std::string const& iEdgeId = iEdge->getID();
                        std::size_t const m = iEdgeId.find_first_of("#");
                        std::string const& iWayId = iEdgeId.substr(0, m);
                        for (size_t j = 0; j < outgoingEdgesNo; j++) {
                            auto const oEdge = outgoingEdges[j];
                            // Searching for a corresponding outgoing edge (based on OSM way identifier)
                            // with at least a pedestrian lane, going in the opposite direction
                            if (oEdge->getID().find(iWayId) != std::string::npos
                                    && oEdge->getSpecialLane(SVC_PEDESTRIAN) > -1
                                    && oEdge->getID().rfind(iWayId, 0) != 0) {
                                EdgeVector edgeVector = EdgeVector{ iEdge };
                                if (oEdge->getFirstNonPedestrianLaneIndex(NBNode::FORWARD) > -1) {
                                    edgeVector.push_back(oEdge);
                                }

                                if (!n->checkCrossingDuplicated(edgeVector)) {
                                    n->addCrossing(edgeVector, crossingWidth, false);
                                }
                            }
                        }
                    }
                }
                for (size_t i = 0; i < outgoingEdgesNo; i++) {
                    // Same checks as above for loop, but for outgoing edges
                    auto const oEdge = outgoingEdges[i];

                    if (oEdge->getFirstNonPedestrianLaneIndex(NBNode::FORWARD) > -1
                            && oEdge->getSpecialLane(SVC_PEDESTRIAN) > -1) {
                        std::string const& oEdgeId = oEdge->getID();
                        std::size_t const m = oEdgeId.find_first_of("#");
                        std::string const& iWayId = oEdgeId.substr(0, m);
                        for (size_t j = 0; j < incomingEdgesNo; j++) {
                            auto const iEdge = incomingEdges[j];
                            if (iEdge->getID().find(iWayId) != std::string::npos
                                    && iEdge->getSpecialLane(SVC_PEDESTRIAN) > -1
                                    && iEdge->getID().rfind(iWayId, 0) != 0) {
                                EdgeVector edgeVector = EdgeVector{ oEdge };
                                if (iEdge->getFirstNonPedestrianLaneIndex(NBNode::FORWARD) > -1) {
                                    edgeVector.push_back(iEdge);
                                }

                                if (!n->checkCrossingDuplicated(edgeVector)) {
                                    n->addCrossing(edgeVector, crossingWidth, false);
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    const double layerElevation = oc.getFloat("osm.layer-elevation");
    if (layerElevation > 0) {
        reconstructLayerElevation(layerElevation, nb);
    }

    // revise pt stops; remove stops on deleted edges
    nb.getPTStopCont().cleanupDeleted(nb.getEdgeCont());

    // load relations (after edges are built since we want to apply
    // turn-restrictions directly to NBEdges)
    RelationHandler relationHandler(myOSMNodes, myEdges, &(nb.getPTStopCont()), myPlatformShapes,
                                    &nb.getPTLineCont(), oc);
    idx = 0;
    for (const std::string& file : files) {
        if (readers[idx] != nullptr) {
            relationHandler.setFileName(file);
            readers[idx]->setHandler(relationHandler);
            const long before = PROGRESS_BEGIN_TIME_MESSAGE("Parsing relations from osm-file '" + file + "'");
            readers[idx]->parseSection(SUMO_TAG_RELATION);
            PROGRESS_TIME_MESSAGE(before);
            delete readers[idx];
        }
        idx++;
    }

    // declare additional stops that are not anchored to a (road)-way or route relation
    std::set<std::string> stopNames;
    for (const auto& item : nb.getPTStopCont().getStops()) {
        stopNames.insert(item.second->getName());
    }
    for (const auto& item : myOSMNodes) {
        const NIOSMNode* n = item.second;
        if (n->ptStopPosition && stopNames.count(n->name) == 0) {
            Position ptPos(n->lon, n->lat, n->ele);
            if (!NBNetBuilder::transformCoordinate(ptPos)) {
                WRITE_ERRORF("Unable to project coordinates for node '%'.", n->id);
            }
            SumoXMLTag element = isRailway(n->permissions) ? SUMO_TAG_TRAIN_STOP : SUMO_TAG_BUS_STOP;
            std::shared_ptr<NBPTStop> ptStop = std::make_shared<NBPTStop>(element, toString(n->id), ptPos, "", "", n->ptStopLength, n->name, n->permissions);
            nb.getPTStopCont().insert(ptStop, true);
        }
    }
}

// ---------------------------------------------------------------------------
// definitions of NIImporter_OpenStreetMap-methods
// ---------------------------------------------------------------------------

NBNode*
NIImporter_OpenStreetMap::insertNodeChecking(long long int id, NBNodeCont& nc, NBTrafficLightLogicCont& tlsc) {
    NBNode* node = nc.retrieve(toString(id));
    if (node == nullptr) {
        NIOSMNode* n = myOSMNodes.find(id)->second;
        Position pos(n->lon, n->lat, n->ele);
        if (!NBNetBuilder::transformCoordinate(pos, true)) {
            WRITE_ERRORF("Unable to project coordinates for junction '%'.", id);
            return nullptr;
        }
        node = new NBNode(toString(id), pos);
        if (!nc.insert(node)) {
            WRITE_ERRORF(TL("Could not insert junction '%'."), toString(id));
            delete node;
            return nullptr;
        }
        n->node = node;
        if (n->railwayCrossing) {
            if (n->getParameter("crossing:barrier", "no") != "no"
                    || n->getParameter("crossing:light") == "yes"
                    || n->getParameter("crossing.light") == "yes"
                    || n->tlsControlled) {
                node->reinit(pos, SumoXMLNodeType::RAIL_CROSSING);
            }
        } else if (n->railwaySignal) {
            node->reinit(pos, SumoXMLNodeType::RAIL_SIGNAL);
        } else if (n->tlsControlled) {
            // ok, this node is a traffic light node where no other nodes
            //  participate
            // @note: The OSM-community has not settled on a schema for differentiating between fixed and actuated lights
            TrafficLightType type = SUMOXMLDefinitions::TrafficLightTypes.get(
                                        OptionsCont::getOptions().getString("tls.default-type"));
            NBOwnTLDef* tlDef = new NBOwnTLDef(toString(id), node, 0, type);
            if (!tlsc.insert(tlDef)) {
                // actually, nothing should fail here
                delete tlDef;
                throw ProcessError(TLF("Could not allocate tls '%'.", toString(id)));
            }
        }
        if (n->railwayBufferStop) {
            node->setParameter("buffer_stop", "true");
            node->setFringeType(FringeType::INNER);
        }
        if (n->railwaySignal) {
            if (n->myRailDirection == WAY_FORWARD) {
                node->setParameter(NBTrafficLightDefinition::OSM_SIGNAL_DIRECTION, "forward");
            } else if (n->myRailDirection == WAY_BACKWARD) {
                node->setParameter(NBTrafficLightDefinition::OSM_SIGNAL_DIRECTION, "backward");
            }
        }
        node->updateParameters(n->getParametersMap());
    }
    return node;
}


int
NIImporter_OpenStreetMap::insertEdge(Edge* e, int index, NBNode* from, NBNode* to,
                                     const std::vector<long long int>& passed, NBNetBuilder& nb,
                                     const NBNode* first, const NBNode* last) {
    NBNodeCont& nc = nb.getNodeCont();
    NBEdgeCont& ec = nb.getEdgeCont();
    NBTypeCont& tc = nb.getTypeCont();
    NBPTStopCont& sc = nb.getPTStopCont();

    NBTrafficLightLogicCont& tlsc = nb.getTLLogicCont();
    // patch the id
    std::string id = toString(e->id);
    if (from == nullptr || to == nullptr) {
        WRITE_ERRORF("Discarding edge '%' because the nodes could not be built.", id);
        return index;
    }
    if (index >= 0) {
        id = id + "#" + toString(index);
    } else {
        index = 0;
    }
    if (from == to) {
        assert(passed.size() >= 2);
        if (passed.size() == 2) {
            WRITE_WARNINGF(TL("Discarding edge '%' which connects two identical nodes without geometry."), id);
            return index;
        }
        // in the special case of a looped way split again using passed
        int intermediateIndex = (int) passed.size() / 2;
        NBNode* intermediate = insertNodeChecking(passed[intermediateIndex], nc, tlsc);
        std::vector<long long int> part1(passed.begin(), passed.begin() + intermediateIndex + 1);
        std::vector<long long int> part2(passed.begin() + intermediateIndex, passed.end());
        index = insertEdge(e, index, from, intermediate, part1, nb, first, last);
        return insertEdge(e, index, intermediate, to, part2, nb, first, last);
    }
    const int newIndex = index + 1;
    const std::string type = usableType(e->myHighWayType, id, tc);
    if (type == "") {  // we do not want to import it
        return newIndex;
    }
    std::string routingType = "";
    int numLanesForward = tc.getEdgeTypeNumLanes(type);
    int numLanesBackward = tc.getEdgeTypeNumLanes(type);
    double speed = tc.getEdgeTypeSpeed(type);
    bool defaultsToOneWay = tc.getEdgeTypeIsOneWay(type);
    const SVCPermissions defaultPermissions = tc.getEdgeTypePermissions(type);
    SVCPermissions extra = myImportBikeAccess ? e->myExtraAllowed : (e->myExtraAllowed & ~SVC_BICYCLE);
    const SVCPermissions extraDis = myImportBikeAccess ? e->myExtraDisallowed : (e->myExtraDisallowed & ~SVC_BICYCLE);
    std::vector<SumoXMLAttr> defaults;
    // Conflict resolution: when a class is both allowed and disallowed,
    // explicit per-mode allows (bus=yes, taxi=yes, hgv=yes, etc., tracked
    // in myExplicitlyAllowed) win unconditionally. Implicit allows (e.g.
    // psv=yes incidentally permitting SVC_BUS, or motor_vehicle=yes
    // incidentally permitting SVC_TRUCK while a separate hgv=no disallows
    // it) lose to the disallow.
    const SVCPermissions implicitConflict = extra & extraDis & ~e->myExplicitlyAllowed;
    extra &= ~implicitConflict;
    SVCPermissions permissions = (defaultPermissions & ~extraDis) | extra;
    if (!myImportBikeAccess && permissions == (SVC_PEDESTRIAN | SVC_BICYCLE)
            && (e->myExtraDisallowed & SVC_BICYCLE) != 0
            && (e->myExtraAllowed & SVC_BICYCLE) == 0) {
        // remove bicyle permissions where they affect network building the most
        permissions = SVC_PEDESTRIAN;
        defaultsToOneWay = true;
    }
    if (defaultPermissions == SVC_SHIP) {
        // extra permission apply to the ships operating on the route rather than the waterway
        permissions = defaultPermissions;
    }
    if (defaultsToOneWay && defaultPermissions == SVC_PEDESTRIAN && (permissions & (~SVC_PEDESTRIAN)) != 0) {
        defaultsToOneWay = false;
    }
    if ((permissions & SVC_RAIL) != 0 && e->myExtraTags.count("electrified") != 0) {
        permissions |= (SVC_RAIL_ELECTRIC | SVC_RAIL_FAST);
    }

    // convert the shape
    PositionVector shape;
    double distanceStart = myOSMNodes[passed.front()]->positionMeters;
    double distanceEnd = myOSMNodes[passed.back()]->positionMeters;
    const bool useDistance = distanceStart != std::numeric_limits<double>::max() && distanceEnd != std::numeric_limits<double>::max();
    if (useDistance) {
        // negative sign denotes counting in the other direction
        if (distanceStart < distanceEnd) {
            distanceEnd *= -1;
        } else {
            distanceStart *= -1;
        }
    } else {
        distanceStart = 0;
        distanceEnd = 0;
    }
    // get additional direction information
    int nodeDirection = WAY_UNKNOWN;
    const NIOSMNode* fn = myOSMNodes.find(StringUtils::toLong(from->getID()))->second;
    const NIOSMNode* ft = myOSMNodes.find(StringUtils::toLong(to->getID()))->second;
    if (fn->railwaySignal) {
        nodeDirection |= fn->myRailDirection;
    }
    if (ft->railwaySignal) {
        nodeDirection |= ft->myRailDirection;
    }

    std::vector<std::shared_ptr<NBPTStop> > ptStops;
    for (long long i : passed) {
        NIOSMNode* n = myOSMNodes.find(i)->second;
        // recheck permissions, maybe they got assigned to a strange edge, see #11656
        if (n->ptStopPosition && (n->permissions == 0 || (permissions & n->permissions) != 0)) {
            std::shared_ptr<NBPTStop> existingPtStop = sc.get(toString(n->id));
            if (existingPtStop != nullptr) {
                existingPtStop->registerAdditionalEdge(toString(e->id), id);
            } else {
                Position ptPos(n->lon, n->lat, n->ele);
                if (!NBNetBuilder::transformCoordinate(ptPos)) {
                    WRITE_ERRORF("Unable to project coordinates for node '%'.", n->id);
                }
                SumoXMLTag element = isRailway(n->permissions) ? SUMO_TAG_TRAIN_STOP : SUMO_TAG_BUS_STOP;
                ptStops.push_back(std::make_shared<NBPTStop>(element, toString(n->id), ptPos, id, toString(e->id), n->ptStopLength, n->name, n->permissions));
                sc.insert(ptStops.back());
            }
        }
        if (n->railwaySignal) {
            nodeDirection |= n->myRailDirection;
        }
        Position pos(n->lon, n->lat, n->ele);
        shape.push_back(pos);
    }
    //if (e->id == DEBUGID) {
    //    std::cout
    //            << " id=" << id << " from=" << from->getID() << " fromRailDirection=" << myOSMNodes.find(StringUtils::toLong(from->getID()))->second->myRailDirection
    //            << " to=" << to->getID() << " toRailDirection=" << myOSMNodes.find(StringUtils::toLong(to->getID()))->second->myRailDirection
    //            << " origRailDirection=" << e->myRailDirection
    //            << " nodeDirection=" << nodeDirection
    //            << "\n";
    //}
    if (e->myRailDirection == WAY_UNKNOWN && (nodeDirection & WAY_BACKWARD) != 0) {
        // legacy behavior seems to have handled missing tags quite well
        e->myRailDirection = WAY_BOTH;
        //std::cout << " id=" << id << " newRailDir=" << e->myRailDirection << "\n";
    } else if (nodeDirection != WAY_UNKNOWN) {
        // additional direction information can just be added
        e->myRailDirection = (e->myRailDirection | nodeDirection) & ~WAY_UNKNOWN;
    }

    if (!NBNetBuilder::transformCoordinates(shape)) {
        WRITE_ERRORF("Unable to project coordinates for edge '%'.", id);
    }

    SVCPermissions forwardPermissions = permissions;
    SVCPermissions backwardPermissions = permissions;
    const std::string streetName = isRailway(permissions) && e->ref != "" ? e->ref : e->streetName;
    if (streetName == e->ref) {
        e->unsetParameter("ref"); // avoid superfluous param for railways
    }
    double forwardWidth = tc.getEdgeTypeWidth(type);
    double backwardWidth = tc.getEdgeTypeWidth(type);
    double sidewalkWidth = tc.getEdgeTypeSidewalkWidth(type);
    bool addSidewalk = sidewalkWidth != NBEdge::UNSPECIFIED_WIDTH;
    if (myImportSidewalks) {
        if (addSidewalk) {
            // only use sidewalk width from typemap but don't add sidewalks
            // unless OSM specifies them
            addSidewalk = false;
        } else {
            sidewalkWidth = OptionsCont::getOptions().getFloat("default.sidewalk-width");
        }
    }
    double bikeLaneWidth = tc.getEdgeTypeBikeLaneWidth(type);
    const std::string& onewayBike = e->myExtraTags["oneway:bicycle"];
    if (onewayBike == "false" || onewayBike == "no" || onewayBike == "0") {
        e->myCyclewayType = e->myCyclewayType == WAY_UNKNOWN ? WAY_BACKWARD : (WayType)(e->myCyclewayType | WAY_BACKWARD);
    }

    const bool addBikeLane = bikeLaneWidth != NBEdge::UNSPECIFIED_WIDTH ||
                             (myImportBikeAccess && (((e->myCyclewayType & WAY_BOTH) != 0 || e->myExtraTags.count("segregated") != 0) &&
                                     !(e->myCyclewayType == WAY_BACKWARD && (e->myBuswayType & WAY_BOTH) != 0)));
    if (addBikeLane && bikeLaneWidth == NBEdge::UNSPECIFIED_WIDTH) {
        bikeLaneWidth = OptionsCont::getOptions().getFloat("default.bikelane-width");
    }
    // check directions
    bool addForward = true;
    bool addBackward = true;
    const bool explicitOneWay = StringUtils::isBool(e->myIsOneWay) && StringUtils::toBool(e->myIsOneWay);
    const bool explicitTwoWay = StringUtils::isBool(e->myIsOneWay) && !StringUtils::toBool(e->myIsOneWay);
    if ((explicitOneWay || (defaultsToOneWay && (!explicitTwoWay || isRailway(permissions)))) && (e->myRailDirection & WAY_BACKWARD) == 0) {
        addBackward = false;
    }
    if (e->myIsOneWay == "-1" || e->myIsOneWay == "reverse"
            || ((e->myRailDirection & WAY_BACKWARD) != 0 && (e->myRailDirection & WAY_FORWARD) == 0)) {
        // one-way in reversed direction of way
        addForward = false;
        addBackward = true;
    }
    if (!e->myIsOneWay.empty() && !explicitOneWay && !explicitTwoWay && e->myIsOneWay != "-1" && e->myIsOneWay != "reverse") {
        WRITE_WARNINGF(TL("New value for oneway found: %"), e->myIsOneWay);
    }
    if ((permissions == SVC_BICYCLE || permissions == (SVC_BICYCLE | SVC_PEDESTRIAN) || permissions == SVC_PEDESTRIAN)) {
        if (addBackward && (onewayBike == "true" || onewayBike == "yes" || onewayBike == "1")) {
            addBackward = false;
        }
        if (addForward && (onewayBike == "reverse" || onewayBike == "-1")) {
            addForward = false;
        }
        if (!addBackward && (onewayBike == "false" || onewayBike == "no" || onewayBike == "0")) {
            addBackward = true;
        }
    }

    // deal with busways that run in the opposite direction of a one-way street before lane allocation
    if (!addForward && ((e->myBuswayType & WAY_FORWARD) != 0 || e->myBusLanesForwardCount > 0)) {
        addForward = true;
        forwardPermissions = (e->myBusLanesForwardClasses != 0) ? e->myBusLanesForwardClasses : (SVCPermissions)SVC_BUS;
    }
    if (!addBackward && ((e->myBuswayType & WAY_BACKWARD) != 0 || e->myBusLanesBackwardCount > 0)) {
        addBackward = true;
        backwardPermissions = (e->myBusLanesBackwardClasses != 0) ? e->myBusLanesBackwardClasses : (SVCPermissions)SVC_BUS;
    }

    // if we had been able to extract the number of lanes, override the highway type default
    if (e->myNoLanes > 0) {
        if (addForward && !addBackward) {
            numLanesForward = e->myNoLanesForwardExplicit > 0 ? e->myNoLanesForwardExplicit : e->myNoLanes;
        } else if (!addForward && addBackward) {
            numLanesBackward = e->myNoLanesBackwardExplicit > 0 ? e->myNoLanesBackwardExplicit : e->myNoLanes;
        } else {
            // Both directions present
            if (e->myNoLanesForwardExplicit > 0 && e->myNoLanesBackwardExplicit > 0) {
                numLanesForward = e->myNoLanesForwardExplicit;
                numLanesBackward = e->myNoLanesBackwardExplicit;
            } else if (e->myNoLanesForwardExplicit > 0) {
                numLanesForward = e->myNoLanesForwardExplicit;
                numLanesBackward = MAX2(1, e->myNoLanes - e->myNoLanesForwardExplicit);
            } else if (e->myNoLanesBackwardExplicit > 0) {
                numLanesBackward = e->myNoLanesBackwardExplicit;
                numLanesForward = MAX2(1, e->myNoLanes - e->myNoLanesBackwardExplicit);
            } else if (((e->myBuswayType & WAY_BACKWARD) != 0 || e->myBusLanesBackwardCount > 0) && (e->myBuswayType & WAY_FORWARD) == 0 && e->myBusLanesForwardCount == 0) {
                // Contraflow backward busway: reserve bus lane(s) for backward, remainder for forward
                numLanesBackward = e->myBusLanesBackwardCount > 0 ? e->myBusLanesBackwardCount : 1;
                numLanesForward = MAX2(1, e->myNoLanes - numLanesBackward);
            } else if (((e->myBuswayType & WAY_FORWARD) != 0 || e->myBusLanesForwardCount > 0) && (e->myBuswayType & WAY_BACKWARD) == 0 && e->myBusLanesBackwardCount == 0) {
                // Contraflow forward busway: reserve bus lane(s) for forward, remainder for backward
                numLanesForward = e->myBusLanesForwardCount > 0 ? e->myBusLanesForwardCount : 1;
                numLanesBackward = MAX2(1, e->myNoLanes - numLanesForward);
            } else {
                numLanesForward = (int) std::ceil(e->myNoLanes / 2.0);
                numLanesBackward = e->myNoLanes - numLanesForward;
                // sometimes ways are tagged according to their physical width of a single
                // lane but they are intended for traffic in both directions
                numLanesForward = MAX2(1, numLanesForward);
                numLanesBackward = MAX2(1, numLanesBackward);
            }
        }
    } else if (e->myNoLanes == 0) {
        WRITE_WARNINGF(TL("Skipping edge '%' because it has zero lanes."), id);
        return newIndex;
    } else {
        // the total number of lanes is not known but at least one direction
        if (e->myNoLanesForwardExplicit > 0) {
            numLanesForward = e->myNoLanesForwardExplicit;
        } else if (((e->myBuswayType & WAY_FORWARD) != 0 || e->myBusLanesForwardCount > 0) && (extraDis & SVC_PASSENGER) == 0) {
            // if we have a bus/PSV lane yet cars may drive, this implies at least 1 general lane + N bus lanes
            numLanesForward = MAX2(numLanesForward, MAX2(2, e->myBusLanesForwardCount + 1));
        } else if ((e->myBuswayType & WAY_FORWARD) != 0 || e->myBusLanesForwardCount > 0) {
            numLanesForward = MAX2(numLanesForward, MAX2(1, e->myBusLanesForwardCount));
        }
        if (e->myNoLanesBackwardExplicit > 0) {
            numLanesBackward = e->myNoLanesBackwardExplicit;
        } else if (((e->myBuswayType & WAY_BACKWARD) != 0 || e->myBusLanesBackwardCount > 0) && (extraDis & SVC_PASSENGER) == 0) {
            // if we have a bus/PSV lane yet cars may drive, this implies at least 1 general lane + N bus lanes
            numLanesBackward = MAX2(numLanesBackward, MAX2(2, e->myBusLanesBackwardCount + 1));
        } else if ((e->myBuswayType & WAY_BACKWARD) != 0 || e->myBusLanesBackwardCount > 0) {
            numLanesBackward = MAX2(numLanesBackward, MAX2(1, e->myBusLanesBackwardCount));
        }
        if (myAnnotateDefaults && e->myNoLanesForwardExplicit == 0 && e->myNoLanesBackwardExplicit == 0) {
            defaults.push_back(SUMO_ATTR_NUMLANES);
        }
    }
    // width is meant for raw lane count before adding sidewalks or cycleways
    const int taggedLanes = (addForward ? numLanesForward : 0) + (addBackward ? numLanesBackward : 0);
    if (e->myWidth > 0 && e->myWidthLanesForward.size() == 0 && e->myWidthLanesBackward.size() == 0 && taggedLanes != 0
            && !OptionsCont::getOptions().getBool("ignore-widths")) {
        // width is tagged excluding sidewalks and cycleways
        forwardWidth = e->myWidth / taggedLanes;
        backwardWidth = forwardWidth;
    }

    // if we had been able to extract the maximum speed, override the type's default
    if (e->myMaxSpeed != MAXSPEED_UNGIVEN) {
        speed = e->myMaxSpeed;
    } else if (myAnnotateDefaults) {
        defaults.push_back(SUMO_ATTR_SPEED);
    }
    double speedBackward = speed;
    if (e->myMaxSpeedBackward != MAXSPEED_UNGIVEN) {
        speedBackward = e->myMaxSpeedBackward;
        addBackward = true;
    }
    if (speed <= 0 || speedBackward <= 0) {
        WRITE_WARNINGF(TL("Skipping edge '%' because it has speed %."), id, speed);
        return newIndex;
    }
    // When an OSM way has lanes=1 on a bidirectional carriageway, it
    // represents a single shared physical strip, not two parallel narrow
    // lanes. Under --osm.repair=infer (or aggressive), emit a proper
    // SUMO bidi edge pair (spreadType=center + setBidi(true)). Under
    // default (off/warn), retain the legacy half-width behaviour so
    // existing test fixtures' golden output is unchanged.
    bool bidiPair = false;
    if (e->myNoLanes == 1 && addForward && addBackward) {
        const std::string repair = OptionsCont::getOptions().getString("osm.repair");
        if (repair == "infer" || repair == "aggressive") {
            bidiPair = true;
            // Do not halve the widths; both edges will overlap geometrically
            // because spreadType=center, and the bidi flag tells SUMO that
            // only one direction can use the strip at a time.
        } else {
            // Legacy behaviour: two parallel half-width lanes. Wrong on both
            // geometry and capacity, but preserved for backward compatibility.
            if (e->myWidth < 0 && e->myWidthLanesForward.size() == 0 && e->myWidthLanesBackward.size() == 0) {
                if (forwardWidth == NBEdge::UNSPECIFIED_WIDTH) {
                    forwardWidth = SUMO_const_laneWidth;
                }
                if (backwardWidth == NBEdge::UNSPECIFIED_WIDTH) {
                    backwardWidth = SUMO_const_laneWidth;
                }
                forwardWidth /= 2;
                backwardWidth /= 2;
            }
        }
        if (e->myWidth < 5) {
            routingType = "narrow";
        }
    }
    // deal with cycleways that run in the opposite direction of a one-way street
    WayType cyclewayType = e->myCyclewayType; // make a copy because we do some temporary modifications
    if (addBikeLane) {
        if (!addForward && (cyclewayType & WAY_FORWARD) != 0) {
            addForward = true;
            forwardPermissions = SVC_BICYCLE;
            forwardWidth = bikeLaneWidth;
            numLanesForward = 1;
            // do not add an additional cycle lane
            cyclewayType = (WayType)(cyclewayType & ~WAY_FORWARD);
        }
        if (!addBackward && (cyclewayType & WAY_BACKWARD) != 0) {
            addBackward = true;
            backwardPermissions = SVC_BICYCLE;
            backwardWidth = bikeLaneWidth;
            numLanesBackward = 1;
            // do not add an additional cycle lane
            cyclewayType = (WayType)(cyclewayType & ~WAY_BACKWARD);
        }
    }
    // deal with sidewalks that run in the opposite direction of a one-way street
    WayType sidewalkType = e->mySidewalkType; // make a copy because we do some temporary modifications
    if (sidewalkType == WAY_UNKNOWN && (e->myExtraAllowed & SVC_PEDESTRIAN) != 0 && (permissions & SVC_PASSENGER) != 0) {
        // do not assume shared space unless sidewalk is actively disabled
        if (myOnewayDualSidewalk) {
            sidewalkType = WAY_BOTH;
        }
    }
    if (addSidewalk || (myImportSidewalks && (permissions & SVC_ROAD_CLASSES) != 0 && defaultPermissions != SVC_PEDESTRIAN)) {
        if (!addForward && (sidewalkType & WAY_FORWARD) != 0) {
            addForward = true;
            forwardPermissions = SVC_PEDESTRIAN;
            forwardWidth = tc.getEdgeTypeSidewalkWidth(type);
            numLanesForward = 1;
            // do not add an additional sidewalk
            sidewalkType = (WayType)(sidewalkType & ~WAY_FORWARD);  //clang tidy thinks "!WAY_FORWARD" is always false
        } else if (addSidewalk && addForward && (sidewalkType & WAY_BOTH) == 0
                   && numLanesForward == 1 && numLanesBackward <= 1
                   && (e->myExtraDisallowed & SVC_PEDESTRIAN) == 0) {
            // our typemap says pedestrians should walk here but the data says
            // there is no sidewalk at all. If the road is small, pedestrians can just walk
            // on the road
            forwardPermissions |= SVC_PEDESTRIAN;
        }
        if (!addBackward && (sidewalkType & WAY_BACKWARD) != 0) {
            addBackward = true;
            backwardPermissions = SVC_PEDESTRIAN;
            backwardWidth = tc.getEdgeTypeSidewalkWidth(type);
            numLanesBackward = 1;
            // do not add an additional cycle lane
            sidewalkType = (WayType)(sidewalkType & ~WAY_BACKWARD); //clang tidy thinks "!WAY_BACKWARD" is always false
        } else if (addSidewalk && addBackward && (sidewalkType & WAY_BOTH) == 0
                   && numLanesBackward == 1 && numLanesForward <= 1
                   && (e->myExtraDisallowed & SVC_PEDESTRIAN) == 0) {
            // our typemap says pedestrians should walk here but the data says
            // there is no sidewalk at all. If the road is small, pedestrians can just walk
            // on the road
            backwardPermissions |= SVC_PEDESTRIAN;
        }
    }

    bool applyPlacement = false;
    double placementOffset = 0;
    if (e->myPlacement != PlacementType::NONE) {
        if (!explicitOneWay) {
            if (index <= 0) {
                myPlacementSkippedNonExplicitOneWay++;
            }
        } else if (!(addForward && !addBackward)) {
            if (index <= 0) {
                myPlacementSkippedAuxOppositeDirection++;
            }
        } else if (numLanesForward <= 0) {
            if (index <= 0) {
                WRITE_WARNINGF(TL("Ignoring placement for edge '%' because lane count is invalid."), id);
            }
        } else {
            const double defaultPlacementWidth = forwardWidth == NBEdge::UNSPECIFIED_WIDTH || forwardWidth <= 0
                                                 ? SUMO_const_laneWidth : forwardWidth;
            std::vector<double> laneWidths((size_t)numLanesForward, defaultPlacementWidth);
            if (!OptionsCont::getOptions().getBool("ignore-widths")
                    && (int)e->myWidthLanesForward.size() == numLanesForward) {
                for (int i = 0; i < numLanesForward; ++i) {
                    laneWidths[(size_t)i] = e->myWidthLanesForward[(size_t)i] > 0
                                            ? e->myWidthLanesForward[(size_t)i] : defaultPlacementWidth;
                }
            }
            if (e->myPlacementLane < 1 || e->myPlacementLane > numLanesForward) {
                if (index <= 0) {
                    WRITE_WARNINGF(TL("Ignoring placement for edge '%' because lane index '%' is out of range [1, %]."),
                                   id, e->myPlacementLane, numLanesForward);
                }
            } else {
                const int laneIndex = e->myPlacementLane - 1;
                double leftOffset = 0;
                for (int i = 0; i < laneIndex; ++i) {
                    leftOffset += laneWidths[(size_t)i];
                }
                double placementRefOffset = leftOffset;
                if (e->myPlacement == PlacementType::RIGHT_OF) {
                    placementRefOffset += laneWidths[(size_t)laneIndex];
                } else if (e->myPlacement == PlacementType::MIDDLE_OF) {
                    placementRefOffset += laneWidths[(size_t)laneIndex] / 2.;
                }
                double totalWidth = 0;
                for (double laneWidth : laneWidths) {
                    totalWidth += laneWidth;
                }
                placementOffset = totalWidth / 2. - placementRefOffset;
                applyPlacement = true;
            }
        }
    }
    if (applyPlacement && fabs(placementOffset) > POSITION_EPS) {
        try {
            shape.move2side(placementOffset);
        } catch (InvalidArgument&) {
            if (index <= 0) {
                WRITE_WARNINGF(TL("Ignoring placement for edge '%' because offset shape computation failed."), id);
            }
            applyPlacement = false;
        }
    }

    const std::string origID = OptionsCont::getOptions().getBool("output.original-names") ? toString(e->id) : "";
    const bool lefthand = OptionsCont::getOptions().getBool("lefthand");
    const int offsetFactor = lefthand ? -1 : 1;
    LaneSpreadFunction lsf = ((addBackward || OptionsCont::getOptions().getBool("osm.oneway-spread-right")) &&
            ((!isRailway(permissions) || (permissions == SVC_CABLE_CAR && e->myRailDirection == WAY_UNKNOWN)))
            ? LaneSpreadFunction::RIGHT : LaneSpreadFunction::CENTER);
    if (addBackward && lsf == LaneSpreadFunction::RIGHT && OptionsCont::getOptions().getString("default.spreadtype") == toString(LaneSpreadFunction::ROADCENTER)) {
        lsf = LaneSpreadFunction::ROADCENTER;
    }
    if (addForward && addBackward && lsf == LaneSpreadFunction::RIGHT && explicitOneWay) {
        lsf = LaneSpreadFunction::ROADCENTER;
    }
    if (tc.getEdgeTypeSpreadType(type) != LaneSpreadFunction::SPREAD_UNKNOWN) {
        // user defined value overrides defaults
        lsf = tc.getEdgeTypeSpreadType(type);
    }
    if (applyPlacement) {
        // placement references the directional edge centerline for one-way edges
        lsf = LaneSpreadFunction::CENTER;
    }
    if (bidiPair) {
        // Both edges of a bidi pair share the same geometry, drawn centred
        // on the way reference line.
        lsf = LaneSpreadFunction::CENTER;
    }
    if (defaults.size() > 0) {
        e->setParameter("osmDefaults", joinToString(defaults, " "));
    }

    id = StringUtils::escapeXML(id);
    const std::string reverseID = "-" + id;
    const bool markOSMDirection =  from->getType() == SumoXMLNodeType::RAIL_SIGNAL || to->getType() == SumoXMLNodeType::RAIL_SIGNAL;
    if (addForward) {
        assert(numLanesForward > 0);
        NBEdge* nbe = new NBEdge(id, from, to, type, speed, NBEdge::UNSPECIFIED_FRICTION, numLanesForward, tc.getEdgeTypePriority(type),
                                 forwardWidth, NBEdge::UNSPECIFIED_OFFSET, shape, lsf,
                                 StringUtils::escapeXML(streetName), origID, true);
        if (markOSMDirection) {
            nbe->setParameter(NBTrafficLightDefinition::OSM_DIRECTION, "forward");
        }
        nbe->setPermissions(forwardPermissions, -1);
        if ((e->myBuswayType & WAY_FORWARD) != 0) {
            nbe->setPermissions(SVC_BUS, 0);
        }
        applyChangeProhibition(nbe, e->myChangeForward);
        applyLaneUse(nbe, e, true);
        applyTurnSigns(nbe, e->myTurnSignsForward);
        nbe->setTurnSignTarget(last->getID());
        if (addBikeLane && (cyclewayType == WAY_UNKNOWN || (cyclewayType & WAY_FORWARD) != 0)) {
            nbe->addBikeLane(bikeLaneWidth * offsetFactor);
        } else if (nbe->getPermissions(0) == SVC_BUS) {
            // bikes drive on buslanes if no separate cycle lane is available
            nbe->setPermissions(SVC_BUS | SVC_BICYCLE, 0);
        }
        if ((addSidewalk && (sidewalkType == WAY_UNKNOWN || (sidewalkType & WAY_FORWARD) != 0))
                || (myImportSidewalks && (sidewalkType & WAY_FORWARD) != 0 && defaultPermissions != SVC_PEDESTRIAN)) {
            nbe->addSidewalk(sidewalkWidth * offsetFactor);
        }
        if (!addBackward && (e->myExtraAllowed & SVC_PEDESTRIAN) != 0 && (nbe->getPermissions(0) & SVC_PEDESTRIAN) == 0) {
            // Pedestrians are explicitly allowed (maybe through foot="yes") but did not get a sidewalk (maybe through sidewalk="no").
            // Since we do not have a backward edge, we need to make sure they can at least walk somewhere, see #14124
            nbe->setPermissions(nbe->getPermissions(0) | SVC_PEDESTRIAN, 0);
        }
        nbe->updateParameters(e->getParametersMap());
        nbe->setDistance(distanceStart);
        if (e->myAmInRoundabout) {
            // ensure roundabout edges have the precedence
            nbe->setJunctionPriority(to, NBEdge::JunctionPriority::ROUNDABOUT);
            nbe->setJunctionPriority(from, NBEdge::JunctionPriority::ROUNDABOUT);
        }

        // process forward lanes width
        const int numForwardLanesFromWidthKey = (int)e->myWidthLanesForward.size();
        const int numForwardLanes = (int)nbe->getLanes().size();
        if (numForwardLanesFromWidthKey > 0 && !OptionsCont::getOptions().getBool("ignore-widths")) {
            if (numForwardLanes != numForwardLanesFromWidthKey) {
                // Apply the entries we have; remaining lanes use the default.
                WRITE_WARNINGF(TL("Forward lanes count for edge '%' (%) does not match the number of lanes in width:lanes:forward (%). Applying widths to the first % lanes; remaining lanes use the default."),
                               id, toString(numForwardLanes), toString(numForwardLanesFromWidthKey),
                               toString(MIN2(numForwardLanes, numForwardLanesFromWidthKey)));
            }
            const int numToApply = MIN2(numForwardLanes, numForwardLanesFromWidthKey);
            for (int i = 0; i < numToApply; i++) {
                const double actualWidth = e->myWidthLanesForward[i] <= 0 ? forwardWidth : e->myWidthLanesForward[i];
                const int laneIndex = lefthand ? i : numForwardLanes - i - 1;
                nbe->setLaneWidth(laneIndex, actualWidth);
            }
        }
        if ((e->myRailDirection & WAY_PREFER_FORWARD) != 0 && isRailway(forwardPermissions)) {
            nbe->setRoutingType("4");
        } else {
            nbe->setRoutingType(routingType);
        }
        if (bidiPair) {
            nbe->setBidi(true);
        }
        // Apply per-lane maxspeed overrides for the forward direction.
        if (!e->mySpeedLanesForward.empty()) {
            const int numForwardLanes = (int)nbe->getLanes().size();
            const int numToApply = MIN2(numForwardLanes, (int)e->mySpeedLanesForward.size());
            for (int i = 0; i < numToApply; i++) {
                const double laneSpeed = e->mySpeedLanesForward[i];
                if (laneSpeed != MAXSPEED_UNGIVEN && laneSpeed > 0) {
                    const int laneIndex = lefthand ? i : numForwardLanes - i - 1;
                    nbe->setSpeed(laneIndex, laneSpeed);
                }
            }
        }

        if (!ec.insert(nbe)) {
            delete nbe;
            throw ProcessError(TLF("Could not add edge '%'.", id));
        }
    }
    if (addBackward) {
        assert(numLanesBackward > 0);
        NBEdge* nbe = new NBEdge(reverseID, to, from, type, speedBackward, NBEdge::UNSPECIFIED_FRICTION, numLanesBackward, tc.getEdgeTypePriority(type),
                                 backwardWidth, NBEdge::UNSPECIFIED_OFFSET, shape.reverse(), lsf,
                                 StringUtils::escapeXML(streetName), origID, true);
        if (markOSMDirection) {
            nbe->setParameter(NBTrafficLightDefinition::OSM_DIRECTION, "backward");
        }
        nbe->setPermissions(backwardPermissions);
        if ((e->myBuswayType & WAY_BACKWARD) != 0) {
            nbe->setPermissions(SVC_BUS, 0);
        }
        applyChangeProhibition(nbe, e->myChangeBackward);
        applyLaneUse(nbe, e, false);
        applyTurnSigns(nbe, e->myTurnSignsBackward);
        nbe->setTurnSignTarget(first->getID());
        if (addBikeLane && (cyclewayType == WAY_UNKNOWN || (cyclewayType & WAY_BACKWARD) != 0)) {
            nbe->addBikeLane(bikeLaneWidth * offsetFactor);
        } else if (nbe->getPermissions(0) == SVC_BUS) {
            // bikes drive on buslanes if no separate cycle lane is available
            nbe->setPermissions(SVC_BUS | SVC_BICYCLE, 0);
        }
        if ((addSidewalk && (sidewalkType == WAY_UNKNOWN || (sidewalkType & WAY_BACKWARD) != 0))
                || (myImportSidewalks && (sidewalkType & WAY_BACKWARD) != 0 && defaultPermissions != SVC_PEDESTRIAN)) {
            nbe->addSidewalk(sidewalkWidth * offsetFactor);
        }
        nbe->updateParameters(e->getParametersMap());
        nbe->setDistance(distanceEnd);
        if (e->myAmInRoundabout) {
            // ensure roundabout edges have the precedence
            nbe->setJunctionPriority(from, NBEdge::JunctionPriority::ROUNDABOUT);
            nbe->setJunctionPriority(to, NBEdge::JunctionPriority::ROUNDABOUT);
        }
        // process backward lanes width
        const int numBackwardLanesFromWidthKey = (int)e->myWidthLanesBackward.size();
        const int numBackwardLanes = (int)nbe->getLanes().size();
        if (numBackwardLanesFromWidthKey > 0 && !OptionsCont::getOptions().getBool("ignore-widths")) {
            if (numBackwardLanes != numBackwardLanesFromWidthKey) {
                // Apply the entries we have; remaining lanes use the default.
                WRITE_WARNINGF(TL("Backward lanes count for edge '%' (%) does not match the number of lanes in width:lanes:backward (%). Applying widths to the first % lanes; remaining lanes use the default."),
                               id, toString(numBackwardLanes), toString(numBackwardLanesFromWidthKey),
                               toString(MIN2(numBackwardLanes, numBackwardLanesFromWidthKey)));
            }
            const int numToApply = MIN2(numBackwardLanes, numBackwardLanesFromWidthKey);
            for (int i = 0; i < numToApply; i++) {
                const double actualWidth = e->myWidthLanesBackward[i] <= 0 ? backwardWidth : e->myWidthLanesBackward[i];
                const int laneIndex = lefthand ? i : numBackwardLanes - i - 1;
                nbe->setLaneWidth(laneIndex, actualWidth);
            }
        }
        if ((e->myRailDirection & WAY_PREFER_BACKWARD) != 0 && isRailway(backwardPermissions)) {
            nbe->setRoutingType("4");
        } else {
            nbe->setRoutingType(routingType);
        }
        if (bidiPair) {
            nbe->setBidi(true);
        }
        // Apply per-lane maxspeed overrides for the backward direction.
        if (!e->mySpeedLanesBackward.empty()) {
            const int numBackwardLanes = (int)nbe->getLanes().size();
            const int numToApply = MIN2(numBackwardLanes, (int)e->mySpeedLanesBackward.size());
            for (int i = 0; i < numToApply; i++) {
                const double laneSpeed = e->mySpeedLanesBackward[i];
                if (laneSpeed != MAXSPEED_UNGIVEN && laneSpeed > 0) {
                    const int laneIndex = lefthand ? i : numBackwardLanes - i - 1;
                    nbe->setSpeed(laneIndex, laneSpeed);
                }
            }
        }

        if (!ec.insert(nbe)) {
            delete nbe;
            throw ProcessError(TLF("Could not add edge '-%'.", id));
        }
    }
    if ((e->myParkingType & PARKING_BOTH) != 0 && OptionsCont::getOptions().isSet("parking-output")) {
        if ((e->myParkingType & PARKING_RIGHT) != 0) {
            if (addForward) {
                nb.getParkingCont().push_back(NBParking(id, id));
            } else {
                /// XXX parking area should be added on the left side of a reverse one-way street
                if ((e->myParkingType & PARKING_LEFT) == 0 && !addBackward) {
                    /// put it on the wrong side (better than nothing)
                    nb.getParkingCont().push_back(NBParking(reverseID, reverseID));
                }
            }
        }
        if ((e->myParkingType & PARKING_LEFT) != 0) {
            if (addBackward) {
                nb.getParkingCont().push_back(NBParking(reverseID, reverseID));
            } else {
                /// XXX parking area should be added on the left side of an one-way street
                if ((e->myParkingType & PARKING_RIGHT) == 0 && !addForward) {
                    /// put it on the wrong side (better than nothing)
                    nb.getParkingCont().push_back(NBParking(id, id));
                }
            }
        }
    }
    return newIndex;
}


void
NIImporter_OpenStreetMap::reconstructLayerElevation(const double layerElevation, NBNetBuilder& nb) {
    NBNodeCont& nc = nb.getNodeCont();
    NBEdgeCont& ec = nb.getEdgeCont();
    // reconstruct elevation from layer info
    // build a map of raising and lowering forces (attractor and distance)
    // for all nodes unknownElevation
    std::map<NBNode*, std::vector<std::pair<double, double> > > layerForces;

    // collect all nodes that belong to a way with layer information
    std::set<NBNode*> knownElevation;
    for (auto& myEdge : myEdges) {
        Edge* e = myEdge.second;
        if (e->myLayer != 0) {
            for (auto j = e->myCurrentNodes.begin(); j != e->myCurrentNodes.end(); ++j) {
                NBNode* node = nc.retrieve(toString(*j));
                if (node != nullptr) {
                    knownElevation.insert(node);
                    layerForces[node].emplace_back(e->myLayer * layerElevation, POSITION_EPS);
                }
            }
        }
    }
#ifdef DEBUG_LAYER_ELEVATION
    std::cout << "known elevations:\n";
    for (std::set<NBNode*>::iterator it = knownElevation.begin(); it != knownElevation.end(); ++it) {
        const std::vector<std::pair<double, double> >& primaryLayers = layerForces[*it];
        std::cout << "  node=" << (*it)->getID() << " ele=";
        for (std::vector<std::pair<double, double> >::const_iterator it_ele = primaryLayers.begin(); it_ele != primaryLayers.end(); ++it_ele) {
            std::cout << it_ele->first << " ";
        }
        std::cout << "\n";
    }
#endif
    // layer data only provides a lower bound on elevation since it is used to
    // resolve the relation among overlapping ways.
    // Perform a sanity check for steep inclines and raise the knownElevation if necessary
    std::map<NBNode*, double> knownEleMax;
    for (auto it : knownElevation) {
        double eleMax = -std::numeric_limits<double>::max();
        const std::vector<std::pair<double, double> >& primaryLayers = layerForces[it];
        for (const auto& primaryLayer : primaryLayers) {
            eleMax = MAX2(eleMax, primaryLayer.first);
        }
        knownEleMax[it] = eleMax;
    }
    const double gradeThreshold = OptionsCont::getOptions().getFloat("osm.layer-elevation.max-grade") / 100;
    bool changed = true;
    while (changed) {
        changed = false;
        for (auto it = knownElevation.begin(); it != knownElevation.end(); ++it) {
            std::map<NBNode*, std::pair<double, double> > neighbors = getNeighboringNodes(*it,
                    knownEleMax[*it]
                    / gradeThreshold * 3,
                    knownElevation);
            for (auto& neighbor : neighbors) {
                if (knownElevation.count(neighbor.first) != 0) {
                    const double grade = fabs(knownEleMax[*it] - knownEleMax[neighbor.first])
                                         / MAX2(POSITION_EPS, neighbor.second.first);
#ifdef DEBUG_LAYER_ELEVATION
                    std::cout << "   grade at node=" << (*it)->getID() << " ele=" << knownEleMax[*it] << " neigh=" << it_neigh->first->getID() << " neighEle=" << knownEleMax[it_neigh->first] << " grade=" << grade << " dist=" << it_neigh->second.first << " speed=" << it_neigh->second.second << "\n";
#endif
                    if (grade > gradeThreshold * 50 / 3.6 / neighbor.second.second) {
                        // raise the lower node to the higher level
                        const double eleMax = MAX2(knownEleMax[*it], knownEleMax[neighbor.first]);
                        if (knownEleMax[*it] < eleMax) {
                            knownEleMax[*it] = eleMax;
                        } else {
                            knownEleMax[neighbor.first] = eleMax;
                        }
                        changed = true;
                    }
                }
            }
        }
    }

    // collect all nodes within a grade-dependent range around knownElevation-nodes and apply knowElevation forces
    std::set<NBNode*> unknownElevation;
    for (auto it = knownElevation.begin(); it != knownElevation.end(); ++it) {
        const double eleMax = knownEleMax[*it];
        const double maxDist = fabs(eleMax) * 100 / layerElevation;
        std::map<NBNode*, std::pair<double, double> > neighbors = getNeighboringNodes(*it, maxDist, knownElevation);
        for (auto& neighbor : neighbors) {
            if (knownElevation.count(neighbor.first) == 0) {
                unknownElevation.insert(neighbor.first);
                layerForces[neighbor.first].emplace_back(eleMax, neighbor.second.first);
            }
        }
    }

    // apply forces to ground-level nodes (neither in knownElevation nor unknownElevation)
    for (auto it = unknownElevation.begin(); it != unknownElevation.end(); ++it) {
        double eleMax = -std::numeric_limits<double>::max();
        const std::vector<std::pair<double, double> >& primaryLayers = layerForces[*it];
        for (const auto& primaryLayer : primaryLayers) {
            eleMax = MAX2(eleMax, primaryLayer.first);
        }
        const double maxDist = fabs(eleMax) * 100 / layerElevation;
        std::map<NBNode*, std::pair<double, double> > neighbors = getNeighboringNodes(*it, maxDist, knownElevation);
        for (auto& neighbor : neighbors) {
            if (knownElevation.count(neighbor.first) == 0 && unknownElevation.count(neighbor.first) == 0) {
                layerForces[*it].emplace_back(0, neighbor.second.first);
            }
        }
    }
    // compute the elevation for each node as the weighted average of all forces
#ifdef DEBUG_LAYER_ELEVATION
    std::cout << "summation of forces\n";
#endif
    std::map<NBNode*, double> nodeElevation;
    for (auto& layerForce : layerForces) {
        const std::vector<std::pair<double, double> >& forces = layerForce.second;
        if (knownElevation.count(layerForce.first) != 0) {
            // use the maximum value
            /*
            double eleMax = -std::numeric_limits<double>::max();
            for (std::vector<std::pair<double, double> >::const_iterator it_force = forces.begin(); it_force != forces.end(); ++it_force) {
                eleMax = MAX2(eleMax, it_force->first);
            }
            */
#ifdef DEBUG_LAYER_ELEVATION
            std::cout << "   node=" << it->first->getID() << " knownElevation=" << knownEleMax[it->first] << "\n";
#endif
            nodeElevation[layerForce.first] = knownEleMax[layerForce.first];
        } else if (forces.size() == 1) {
            nodeElevation[layerForce.first] = forces.front().first;
        } else {
            // use the weighted sum
            double distSum = 0;
            for (const auto& force : forces) {
                distSum += force.second;
            }
            double weightSum = 0;
            double elevation = 0;
#ifdef DEBUG_LAYER_ELEVATION
            std::cout << "   node=" << it->first->getID() << "  distSum=" << distSum << "\n";
#endif
            for (const auto& force : forces) {
                const double weight = (distSum - force.second) / distSum;
                weightSum += weight;
                elevation += force.first * weight;

#ifdef DEBUG_LAYER_ELEVATION
                std::cout << "       force=" << it_force->first << " dist=" << it_force->second << "  weight=" << weight << " ele=" << elevation << "\n";
#endif
            }
            nodeElevation[layerForce.first] = elevation / weightSum;
        }
    }
#ifdef DEBUG_LAYER_ELEVATION
    std::cout << "final elevations:\n";
    for (std::map<NBNode*, double>::iterator it = nodeElevation.begin(); it != nodeElevation.end(); ++it) {
        std::cout << "  node=" << (it->first)->getID() << " ele=" << it->second << "\n";
    }
#endif
    // apply node elevations
    for (auto& it : nodeElevation) {
        NBNode* n = it.first;
        n->reinit(n->getPosition() + Position(0, 0, it.second), n->getType());
    }

    // apply way elevation to all edges that had layer information
    for (const auto& it : ec) {
        NBEdge* edge = it.second;
        const PositionVector& geom = edge->getGeometry();
        const double length = geom.length2D();
        const double zFrom = nodeElevation[edge->getFromNode()];
        const double zTo = nodeElevation[edge->getToNode()];
        // XXX if the from- or to-node was part of multiple ways with
        // different layers, reconstruct the layer value from origID
        double dist = 0;
        PositionVector newGeom;
        for (auto it_pos = geom.begin(); it_pos != geom.end(); ++it_pos) {
            if (it_pos != geom.begin()) {
                dist += (*it_pos).distanceTo2D(*(it_pos - 1));
            }
            newGeom.push_back((*it_pos) + Position(0, 0, zFrom + (zTo - zFrom) * dist / length));
        }
        edge->setGeometry(newGeom);
    }
}

std::map<NBNode*, std::pair<double, double> >
NIImporter_OpenStreetMap::getNeighboringNodes(NBNode* node, double maxDist, const std::set<NBNode*>& knownElevation) {
    std::map<NBNode*, std::pair<double, double> > result;
    std::set<NBNode*> visited;
    std::vector<NBNode*> open;
    open.push_back(node);
    result[node] = std::make_pair(0, 0);
    while (!open.empty()) {
        NBNode* n = open.back();
        open.pop_back();
        if (visited.count(n) != 0) {
            continue;
        }
        visited.insert(n);
        const EdgeVector& edges = n->getEdges();
        for (auto e : edges) {
            NBNode* s = nullptr;
            if (n->hasIncoming(e)) {
                s = e->getFromNode();
            } else {
                s = e->getToNode();
            }
            const double dist = result[n].first + e->getGeometry().length2D();
            const double speed = MAX2(e->getSpeed(), result[n].second);
            if (result.count(s) == 0) {
                result[s] = std::make_pair(dist, speed);
            } else {
                result[s] = std::make_pair(MIN2(dist, result[s].first), MAX2(speed, result[s].second));
            }
            if (dist < maxDist && knownElevation.count(s) == 0) {
                open.push_back(s);
            }
        }
    }
    result.erase(node);
    return result;
}


std::string
NIImporter_OpenStreetMap::usableType(const std::string& type, const std::string& id, NBTypeCont& tc) {
    if (tc.knows(type)) {
        return type;
    }
    if (myUnusableTypes.count(type) > 0) {
        return "";
    }
    if (myKnownCompoundTypes.count(type) > 0) {
        return myKnownCompoundTypes[type];
    }
    // this edge has a type which does not yet exist in the TypeContainer
    StringTokenizer tok = StringTokenizer(type, compoundTypeSeparator);
    std::vector<std::string> types;
    while (tok.hasNext()) {
        std::string t = tok.next();
        if (tc.knows(t)) {
            if (std::find(types.begin(), types.end(), t) == types.end()) {
                types.push_back(t);
            }
        } else if (tok.size() > 1) {
            if (!StringUtils::startsWith(t, "service.")) {
                WRITE_WARNINGF(TL("Discarding unknown compound '%' in type '%' (first occurrence for edge '%')."), t, type, id);
            }
        }
    }
    if (types.empty()) {
        if (!StringUtils::startsWith(type, "service.")) {
            WRITE_WARNINGF(TL("Discarding unusable type '%' (first occurrence for edge '%')."), type, id);
        }
        myUnusableTypes.insert(type);
        return "";
    }
    const std::string newType = joinToString(types, "|");
    if (tc.knows(newType)) {
        myKnownCompoundTypes[type] = newType;
        return newType;
    } else if (myKnownCompoundTypes.count(newType) > 0) {
        return myKnownCompoundTypes[newType];
    } else {
        // build a new type by merging all values
        int numLanes = 0;
        double maxSpeed = 0;
        int prio = 0;
        double width = NBEdge::UNSPECIFIED_WIDTH;
        double sidewalkWidth = NBEdge::UNSPECIFIED_WIDTH;
        double bikelaneWidth = NBEdge::UNSPECIFIED_WIDTH;
        bool defaultIsOneWay = true;
        SVCPermissions permissions = 0;
        LaneSpreadFunction spreadType = LaneSpreadFunction::RIGHT;
        bool discard = true;
        bool hadDiscard = false;
        for (auto& type2 : types) {
            if (!tc.getEdgeTypeShallBeDiscarded(type2)) {
                numLanes = MAX2(numLanes, tc.getEdgeTypeNumLanes(type2));
                maxSpeed = MAX2(maxSpeed, tc.getEdgeTypeSpeed(type2));
                prio = MAX2(prio, tc.getEdgeTypePriority(type2));
                defaultIsOneWay &= tc.getEdgeTypeIsOneWay(type2);
                //std::cout << "merging component " << type2 << " into type " << newType << " allows=" << getVehicleClassNames(tc.getPermissions(type2)) << " oneway=" << defaultIsOneWay << "\n";
                permissions |= tc.getEdgeTypePermissions(type2);
                spreadType = tc.getEdgeTypeSpreadType(type2);
                width = MAX2(width, tc.getEdgeTypeWidth(type2));
                sidewalkWidth = MAX2(sidewalkWidth, tc.getEdgeTypeSidewalkWidth(type2));
                bikelaneWidth = MAX2(bikelaneWidth, tc.getEdgeTypeBikeLaneWidth(type2));
                discard = false;
            } else {
                hadDiscard = true;
            }
        }
        if (hadDiscard && permissions == 0) {
            discard = true;
        }
        if (discard) {
            WRITE_WARNINGF(TL("Discarding compound type '%' (first occurrence for edge '%')."), newType, id);
            myUnusableTypes.insert(newType);
            return "";
        }
        if (width != NBEdge::UNSPECIFIED_WIDTH) {
            width = MAX2(width, SUMO_const_laneWidth);
        }
        // ensure pedestrians don't run into trains
        if (sidewalkWidth == NBEdge::UNSPECIFIED_WIDTH
                && (permissions & SVC_PEDESTRIAN) != 0
                && (permissions & SVC_RAIL_CLASSES) != 0) {
            //std::cout << "patching sidewalk for type '" << newType << "' which allows=" << getVehicleClassNames(permissions) << "\n";
            sidewalkWidth = OptionsCont::getOptions().getFloat("default.sidewalk-width");
        }

        WRITE_MESSAGEF(TL("Adding new type '%' (first occurrence for edge '%')."), type, id);
        tc.insertEdgeType(newType, numLanes, maxSpeed, prio, permissions, spreadType, width,
                          defaultIsOneWay, sidewalkWidth, bikelaneWidth, 0, 0, 0);
        for (auto& type3 : types) {
            if (!tc.getEdgeTypeShallBeDiscarded(type3)) {
                tc.copyEdgeTypeRestrictionsAndAttrs(type3, newType);
            }
        }
        myKnownCompoundTypes[type] = newType;
        return newType;
    }
}

void
NIImporter_OpenStreetMap::extendRailwayDistances(Edge* e, NBTypeCont& tc) {
    const std::string id = toString(e->id);
    std::string type = usableType(e->myHighWayType, id, tc);
    if (type != "" && isRailway(tc.getEdgeTypePermissions(type))) {
        std::vector<NIOSMNode*> nodes;
        std::vector<double> usablePositions;
        std::vector<int> usableIndex;
        for (long long int n : e->myCurrentNodes) {
            NIOSMNode* node = myOSMNodes[n];
            node->positionMeters = interpretDistance(node);
            if (node->positionMeters != std::numeric_limits<double>::max()) {
                usablePositions.push_back(node->positionMeters);
                usableIndex.push_back((int)nodes.size());
            }
            nodes.push_back(node);
        }
        if (usablePositions.size() == 0) {
            return;
        } else {
            bool forward = true;
            if (usablePositions.size() == 1) {
                WRITE_WARNINGF(TL("Ambiguous railway kilometrage direction for way '%' (assuming forward)"), id);
            } else {
                forward = usablePositions.front() < usablePositions.back();
            }
            // check for consistency
            for (int i = 1; i < (int)usablePositions.size(); i++) {
                if ((usablePositions[i - 1] < usablePositions[i]) != forward) {
                    WRITE_WARNINGF(TL("Inconsistent railway kilometrage direction for way '%': % (skipping)"), id, toString(usablePositions));
                    return;
                }
            }
            if (nodes.size() > usablePositions.size()) {
                // complete missing values
                PositionVector shape;
                for (NIOSMNode* node : nodes) {
                    shape.push_back(Position(node->lon, node->lat, 0));
                }
                if (!NBNetBuilder::transformCoordinates(shape)) {
                    return; // error will be given later
                }
                double sign = forward ? 1 : -1;
                // extend backward before first usable value
                for (int i = usableIndex.front() - 1; i >= 0; i--) {
                    nodes[i]->positionMeters = nodes[i + 1]->positionMeters - sign * shape[i].distanceTo2D(shape[i + 1]);
                }
                // extend forward
                for (int i = usableIndex.front() + 1; i < (int)nodes.size(); i++) {
                    if (nodes[i]->positionMeters == std::numeric_limits<double>::max()) {
                        nodes[i]->positionMeters = nodes[i - 1]->positionMeters + sign * shape[i].distanceTo2D(shape[i - 1]);
                    }
                }
                //std::cout << " way=" << id << " usable=" << toString(usablePositions) << "\n indices=" << toString(usableIndex)
                //    << " final:\n";
                //for (auto n : nodes) {
                //    std::cout << "    " << n->id << " " << n->positionMeters << " " << n->position<< "\n";
                //}
            }
        }
    }
}


double
NIImporter_OpenStreetMap::interpretDistance(NIOSMNode* node) {
    if (node->position.size() > 0) {
        try {
            if (StringUtils::startsWith(node->position, "mi:")) {
                return StringUtils::toDouble(node->position.substr(3)) * 1609.344; // meters per mile
            } else {
                return StringUtils::toDouble(node->position) * 1000;
            }
        } catch (...) {
            WRITE_WARNINGF(TL("Value of railway:position is not numeric ('%') in node '%'."), node->position, toString(node->id));
        }
    }
    return std::numeric_limits<double>::max();
}

SUMOVehicleClass
NIImporter_OpenStreetMap::interpretTransportType(const std::string& type, NIOSMNode* toSet) {
    SUMOVehicleClass result = SVC_IGNORING;
    if (type == "train") {
        result = SVC_RAIL;
    } else if (type == "subway") {
        result = SVC_SUBWAY;
    } else if (type == "aerialway") {
        result = SVC_CABLE_CAR;
    } else if (type == "light_rail" || type == "monorail") {
        result = SVC_RAIL_URBAN;
    } else if (type == "share_taxi") {
        result = SVC_TAXI;
    } else if (type == "minibus") {
        result = SVC_BUS;
    } else if (type == "trolleybus") {
        result = SVC_BUS;
    } else if (SumoVehicleClassStrings.hasString(type)) {
        result = SumoVehicleClassStrings.get(type);
    }
    std::string stop = "";
    if (result == SVC_TRAM) {
        stop = ".tram";
    } else if (result == SVC_BUS) {
        stop = ".bus";
    } else if (isRailway(result)) {
        stop = ".train";
    }
    if (toSet != nullptr && result != SVC_IGNORING) {
        toSet->permissions |= result;
        toSet->ptStopLength = OptionsCont::getOptions().getFloat("osm.stop-output.length" + stop);
    }
    return result;
}


void
NIImporter_OpenStreetMap::applyChangeProhibition(NBEdge* e, int changeProhibition) {
    bool multiLane = changeProhibition > 3;
    //std::cout << "applyChangeProhibition e=" << e->getID() << " changeProhibition=" << std::bitset<32>(changeProhibition) << " val=" << changeProhibition << "\n";
    for (int lane = 0; changeProhibition > 0 && lane < e->getNumLanes(); lane++) {
        int code = changeProhibition % 4; // only look at the last 2 bits
        SVCPermissions changeLeft = (code & CHANGE_NO_LEFT) == 0 ? SVCAll : (SVCPermissions)SVC_AUTHORITY;
        SVCPermissions changeRight = (code & CHANGE_NO_RIGHT) == 0 ? SVCAll : (SVCPermissions)SVC_AUTHORITY;
        e->setPermittedChanging(lane, changeLeft, changeRight);
        if (multiLane) {
            changeProhibition = changeProhibition >> 2;
        }
    }
}


void
NIImporter_OpenStreetMap::applyLaneUse(NBEdge* e, NIImporter_OpenStreetMap::Edge* nie, const bool forward) {
    const std::vector<bool>& designated = forward ? nie->myDesignatedLaneForward : nie->myDesignatedLaneBackward;
    const std::vector<SVCPermissions>& allowed = forward ? nie->myAllowedLaneForward : nie->myAllowedLaneBackward;
    const std::vector<SVCPermissions>& disallowed = forward ? nie->myDisallowedLaneForward : nie->myDisallowedLaneBackward;
    if (!myImportLaneAccess) {
        // Surface a warning instead of silently discarding hand-tagged
        // per-lane access data when the global flag is off. Forward-only
        // so two-way edges don't emit the warning twice.
        if (forward && (!designated.empty() || !allowed.empty() || !disallowed.empty())) {
            WRITE_WARNINGF(TL("Per-lane access tags on edge '%' were parsed but ignored because --osm.lane-access is not set; pass --osm.lane-access to honour them."),
                           e->getID());
        }
        return;
    }
    const int numLanes = e->getNumLanes();
    const bool lefthand = OptionsCont::getOptions().getBool("lefthand");
    const int busCount = forward ? nie->myBusLanesForwardCount : nie->myBusLanesBackwardCount;
    const SVCPermissions busClasses = forward ? nie->myBusLanesForwardClasses : nie->myBusLanesBackwardClasses;
    if (busCount > 0 && designated.empty() && allowed.empty()) {
        for (int lane = 0; lane < MIN2(numLanes, busCount); lane++) {
            e->setPermissions(busClasses, lane);
            e->preferVehicleClass(lane, busClasses);
        }
    }
    for (int lane = 0; lane < numLanes; lane++) {
        // laneUse stores from left to right
        const int i = lefthand ? lane : numLanes - 1 - lane;
        // Extra allowed SVCs for this lane or none if no info was present for the lane
        const SVCPermissions extraAllowed = i < (int)allowed.size() ? allowed[i] : (SVCPermissions)SVC_IGNORING;
        // Extra disallowed SVCs for this lane or none if no info was present for the lane
        const SVCPermissions extraDisallowed = i < (int)disallowed.size() ? disallowed[i] : (SVCPermissions)SVC_IGNORING;
        if (i < (int)designated.size() && designated[i]) {
            // if designated, delete all permissions
            e->setPermissions(SVC_IGNORING, lane);
            e->preferVehicleClass(lane, extraAllowed);
        }
        e->setPermissions((e->getPermissions(lane) | extraAllowed) & (~extraDisallowed), lane);
    }
}

void
NIImporter_OpenStreetMap::mergeTurnSigns(std::vector<int>& signs, std::vector<int> signs2) {
    if (signs.empty()) {
        signs.insert(signs.begin(), signs2.begin(), signs2.end());
    } else {
        for (int i = 0; i < (int)MIN2(signs.size(), signs2.size()); i++) {
            signs[i] |= signs2[i];
        }
    }
}


void
NIImporter_OpenStreetMap::applyTurnSigns(NBEdge* e, const std::vector<int>& turnSigns) {
    if (myImportTurnSigns && turnSigns.size() > 0) {
        // no sidewalks and bike lanes have been added yet
        if ((int)turnSigns.size() == e->getNumLanes()) {
            //std::cout << "apply turnSigns for " << e->getID() << " turnSigns=" << toString(turnSigns) << "\n";
            for (int i = 0; i < (int)turnSigns.size(); i++) {
                // laneUse stores from left to right
                const int laneIndex = e->getNumLanes() - 1 - i;
                NBEdge::Lane& lane = e->getLaneStruct(laneIndex);
                lane.turnSigns = turnSigns[i];
            }
        } else {
            WRITE_WARNINGF(TL("Ignoring turn sign information for % lanes on edge % with % driving lanes"), turnSigns.size(), e->getID(), e->getNumLanes());
        }
    }
}


// ---------------------------------------------------------------------------
// definitions of NIImporter_OpenStreetMap::NodesHandler-methods
// ---------------------------------------------------------------------------
NIImporter_OpenStreetMap::NodesHandler::NodesHandler(std::map<long long int, NIOSMNode*>& toFill,
        std::set<NIOSMNode*, CompareNodes>& uniqueNodes, const OptionsCont& oc) :
    SUMOSAXHandler("osm - file"),
    myToFill(toFill),
    myCurrentNode(nullptr),
    myIsStation(false),
    myHierarchyLevel(0),
    myUniqueNodes(uniqueNodes),
    myImportElevation(oc.getBool("osm.elevation")),
    myDuplicateNodes(0),
    myOptionsCont(oc) {
    // init rail signal rules
    for (std::string kv : oc.getStringVector("osm.railsignals")) {
        if (kv == "DEFAULT") {
            myRailSignalRules.push_back("railway:signal:main=");
            myRailSignalRules.push_back("railway:signal:combined=");
        } else if (kv == "ALL") {
            myRailSignalRules.push_back("railway=signal");
        } else {
            myRailSignalRules.push_back("railway:signal:" + kv);
        }
    }
}


NIImporter_OpenStreetMap::NodesHandler::~NodesHandler() = default;

void
NIImporter_OpenStreetMap::NodesHandler::myStartElement(int element, const SUMOSAXAttributes& attrs) {
    ++myHierarchyLevel;
    if (element == SUMO_TAG_NODE) {
        bool ok = true;
        myLastNodeID = attrs.get<std::string>(SUMO_ATTR_ID, nullptr, ok);
        if (myHierarchyLevel != 2) {
            WRITE_ERROR("Node element on wrong XML hierarchy level (id='" + myLastNodeID +
                        "', level='" + toString(myHierarchyLevel) + "').");
            return;
        }
        const std::string& action = attrs.getOpt<std::string>(SUMO_ATTR_ACTION, myLastNodeID.c_str(), ok);
        if (action == "delete" || !ok) {
            return;
        }
        try {
            // we do not use attrs.get here to save some time on parsing
            const long long int id = StringUtils::toLong(myLastNodeID);
            myCurrentNode = nullptr;
            const auto insertionIt = myToFill.lower_bound(id);
            if (insertionIt == myToFill.end() || insertionIt->first != id) {
                // assume we are loading multiple files, so we won't report duplicate nodes
                const double tlon = attrs.get<double>(SUMO_ATTR_LON, myLastNodeID.c_str(), ok);
                const double tlat = attrs.get<double>(SUMO_ATTR_LAT, myLastNodeID.c_str(), ok);
                if (!ok) {
                    return;
                }
                myCurrentNode = new NIOSMNode(id, tlon, tlat);
                auto similarNode = myUniqueNodes.find(myCurrentNode);
                if (similarNode == myUniqueNodes.end()) {
                    myUniqueNodes.insert(myCurrentNode);
                } else {
                    delete myCurrentNode;
                    myCurrentNode = *similarNode;
                    myDuplicateNodes++;
                }
                myToFill.emplace_hint(insertionIt, id, myCurrentNode);
            }
        } catch (FormatException&) {
            WRITE_ERROR(TL("Attribute 'id' in the definition of a node is not of type long long int."));
            return;
        }
    }
    if (element == SUMO_TAG_TAG && myCurrentNode != nullptr) {
        if (myHierarchyLevel != 3) {
            WRITE_ERROR(TL("Tag element on wrong XML hierarchy level."));
            return;
        }
        bool ok = true;
        const std::string& key = attrs.get<std::string>(SUMO_ATTR_K, myLastNodeID.c_str(), ok, false);
        // we check whether the key is relevant (and we really need to transcode the value) to avoid hitting #1636
        if (key == "highway" || key == "ele" || key == "crossing" || key == "traffic_signals" || key == "railway" || key == "public_transport"
                || key == "name" || key == "train" || key == "bus" || key == "tram" || key == "light_rail" || key == "subway" || key == "station" || key == "noexit"
                || key == "crossing:barrier"
                || key == "crossing:light"
                || key == "railway:ref"
                || StringUtils::startsWith(key, "railway:signal")
                || StringUtils::startsWith(key, "railway:position")
           ) {
            const std::string& value = attrs.get<std::string>(SUMO_ATTR_V, myLastNodeID.c_str(), ok, false);
            const bool discardPedTls = myOptionsCont.getBool("tls.discard-pedestrian-crossing");
            if (key == "highway" && value.find("traffic_signal") != std::string::npos) {
                if (!discardPedTls || (!myCurrentNode->pedestrianCrossing && myCurrentNode->getParameter("traffic_signals") != "pedestrian_crossing")) {
                    myCurrentNode->tlsControlled = true;
                }
            } else if (key == "crossing" && value.find("traffic_signals") != std::string::npos) {
                myCurrentNode->pedestrianCrossing = true;
                if (!discardPedTls) {
                    myCurrentNode->tlsControlled = true;
                }
            } else if (key == "traffic_signals" && value == "pedestrian_crossing") {
                myCurrentNode->pedestrianCrossing = true;
                myCurrentNode->setParameter("traffic_signals", value);
                if (discardPedTls) {
                    myCurrentNode->tlsControlled = false;
                }
            } else if (key == "highway" && value.find("crossing") != std::string::npos) {
                myCurrentNode->pedestrianCrossing = true;
                if (discardPedTls && myCurrentNode->getParameter("traffic_signals") == "pedestrian_crossing") {
                    myCurrentNode->tlsControlled = false;
                }
            } else if ((key == "noexit" && value == "yes")
                       || (key == "railway" && value == "buffer_stop")) {
                myCurrentNode->railwayBufferStop = true;
            } else if (key == "railway" && value.find("crossing") != std::string::npos) {
                myCurrentNode->railwayCrossing = true;
            } else if (key == "crossing:barrier") {
                myCurrentNode->setParameter("crossing:barrier", value);
            } else if (key == "crossing:light") {
                myCurrentNode->setParameter("crossing:light", value);
            } else if (key == "railway:signal:direction") {
                if (value == "both") {
                    myCurrentNode->myRailDirection = WAY_BOTH;
                } else if (value == "backward") {
                    myCurrentNode->myRailDirection = WAY_BACKWARD;
                } else if (value == "forward") {
                    myCurrentNode->myRailDirection = WAY_FORWARD;
                }
            } else if (StringUtils::startsWith(key, "railway:signal") || (key == "railway" && value == "signal")) {
                std::string kv = key + "=" + value;
                std::string kglob = key + "=";
                if ((std::find(myRailSignalRules.begin(), myRailSignalRules.end(), kv) != myRailSignalRules.end())
                        || (std::find(myRailSignalRules.begin(), myRailSignalRules.end(), kglob) != myRailSignalRules.end())) {
                    myCurrentNode->railwaySignal = true;
                }
            } else if (StringUtils::startsWith(key, "railway:position") && value.size() > myCurrentNode->position.size()) {
                // use the entry with the highest precision (more digits)
                myCurrentNode->position = value;
            } else if ((key == "public_transport" && value == "stop_position") ||
                       (key == "highway" && value == "bus_stop")) {
                myCurrentNode->ptStopPosition = true;
                if (myCurrentNode->ptStopLength == 0) {
                    // default length
                    myCurrentNode->ptStopLength = myOptionsCont.getFloat("osm.stop-output.length");
                }
            } else if (key == "name") {
                myCurrentNode->name = value;
            } else if (myImportElevation && key == "ele") {
                try {
                    const double elevation = StringUtils::parseDist(value);
                    if (std::isnan(elevation)) {
                        WRITE_WARNINGF(TL("Value of key '%' is invalid ('%') in node '%'."), key, value, myLastNodeID);
                    } else {
                        myCurrentNode->ele = elevation;
                    }
                } catch (...) {
                    WRITE_WARNINGF(TL("Value of key '%' is not numeric ('%') in node '%'."), key, value, myLastNodeID);
                }
            } else if (key == "station") {
                interpretTransportType(value, myCurrentNode);
                myIsStation = true;
            } else if (key == "railway:ref") {
                myRailwayRef = value;
            } else {
                // v="yes"
                interpretTransportType(key, myCurrentNode);
            }
        }
        if (myAllAttributes && (myExtraAttributes.count(key) != 0 || myExtraAttributes.size() == 0)) {
            const std::string info = "node=" + toString(myCurrentNode->id) + ", k=" + key;
            myCurrentNode->setParameter(key, attrs.get<std::string>(SUMO_ATTR_V, info.c_str(), ok, false));
        }
    }
}


void
NIImporter_OpenStreetMap::NodesHandler::myEndElement(int element) {
    if (element == SUMO_TAG_NODE && myHierarchyLevel == 2) {
        if (myIsStation && myRailwayRef != "") {
            myCurrentNode->setParameter("railway:ref", myRailwayRef);
        }
        myCurrentNode = nullptr;
        myIsStation = false;
        myRailwayRef = "";
    }
    --myHierarchyLevel;
}


// ---------------------------------------------------------------------------
// OSM-tag-evidence observers
// ---------------------------------------------------------------------------
namespace {

/// Pick the highest-confidence witness from a list. Ties: latest added wins.
/// Returns false if the list is empty.
template <typename T>
bool pickHighestConfidence(const std::vector<NIOSMEvidence<T>>& witnesses,
                           T& outValue, std::string& outSource) {
    if (witnesses.empty()) {
        return false;
    }
    auto best = witnesses.begin();
    for (auto it = witnesses.begin() + 1; it != witnesses.end(); ++it) {
        if (it->confidence >= best->confidence) {
            best = it;
        }
    }
    outValue = best->value;
    outSource = best->sourceTag;
    return true;
}

/// Format every witness in a list as "value (source)" joined by ", ".
template <typename T>
std::string describeWitnesses(const std::vector<NIOSMEvidence<T>>& witnesses) {
    std::ostringstream s;
    bool first = true;
    for (const auto& w : witnesses) {
        if (!first) {
            s << ", ";
        }
        s << w.value << " (" << w.sourceTag << ")";
        first = false;
    }
    return s.str();
}

/// Quick ISO-date well-formedness check. Accepts YYYY, YYYY-MM, or
/// YYYY-MM-DD. Rejects anything else (fuzzy values like "summer 2026",
/// "~2025", or open-ended ranges).
bool isLikelyIsoDate(const std::string& d) {
    if (d.size() == 4 || d.size() == 7 || d.size() == 10) {
        for (size_t i = 0; i < d.size(); ++i) {
            if (i == 4 || i == 7) {
                if (d[i] != '-') {
                    return false;
                }
            } else {
                if (d[i] < '0' || d[i] > '9') {
                    return false;
                }
            }
        }
        return true;
    }
    return false;
}

/// Pad an ISO date to YYYY-MM-DD form, taking the earliest interpretation
/// for partial dates (so "2025" becomes "2025-01-01"). Used for start_date.
std::string padIsoDateEarliest(const std::string& d) {
    if (d.size() == 4) {
        return d + "-01-01";
    }
    if (d.size() == 7) {
        return d + "-01";
    }
    return d;
}

/// Pad an ISO date to YYYY-MM-DD form, taking the latest interpretation
/// for partial dates ("2025" -> "2025-12-31"). Used for end_date.
std::string padIsoDateLatest(const std::string& d) {
    if (d.size() == 4) {
        return d + "-12-31";
    }
    if (d.size() == 7) {
        // overshoot slightly to "31" — fine for lexicographic compares since
        // calendar months never extend beyond -31
        return d + "-31";
    }
    return d;
}

/// Date-window status for an edge given the configured --osm.date.
enum class DateStatus {
    /// No --osm.date configured, or no date tags on the edge.
    NotConfigured,
    /// Edge's [start_date, end_date] contains the simulated date.
    InRange,
    /// Edge has at least one date tag and the simulated date is outside.
    OutOfRange
};

/// Decide whether the edge falls within the configured simulated date.
template <typename EdgeT>
DateStatus checkDateWindow(const EdgeT* e, const std::string& simulatedDate) {
    if (simulatedDate.empty()) {
        return DateStatus::NotConfigured;
    }
    if (e->myStartDate.empty() && e->myEndDate.empty()) {
        return DateStatus::NotConfigured;
    }
    const std::string normSimDateEarliest = padIsoDateEarliest(simulatedDate);
    const std::string normSimDateLatest = padIsoDateLatest(simulatedDate);
    if (!e->myStartDate.empty()) {
        if (!isLikelyIsoDate(e->myStartDate)) {
            WRITE_WARNINGF(TL("Edge '%' has unparseable start_date=%; skipping date filter for this way."),
                           toString(e->id), e->myStartDate);
            return DateStatus::NotConfigured;
        }
        if (normSimDateLatest < padIsoDateEarliest(e->myStartDate)) {
            return DateStatus::OutOfRange;
        }
    }
    if (!e->myEndDate.empty()) {
        if (!isLikelyIsoDate(e->myEndDate)) {
            WRITE_WARNINGF(TL("Edge '%' has unparseable end_date=%; skipping date filter for this way."),
                           toString(e->id), e->myEndDate);
            return DateStatus::NotConfigured;
        }
        if (normSimDateEarliest > padIsoDateLatest(e->myEndDate)) {
            return DateStatus::OutOfRange;
        }
    }
    return DateStatus::InRange;
}

/// Interpret an OSM oneway-tag value string. Returns true for the values
/// that mean "this way carries traffic in only one direction"
/// (yes/true/1/-1/reverse), false for bidirectional values
/// (no/false/0/empty), and falls back to false for anything unrecognized.
bool isOnewayValue(const std::string& v) {
    return v == "yes" || v == "true" || v == "1"
        || v == "-1" || v == "reverse";
}

/// Lane-count-balance observer. Checks that the highest-confidence witnesses
/// for lanesTotal, lanesForward, lanesBackward and lanesBothWays satisfy the
/// conservation law:
///     lanesTotal = lanesForward + lanesBackward + lanesBothWays
/// Emits a warning when (a) at least three of the four are witnessed, AND
/// (b) the sum disagrees with the witnessed total. Pure observation: does
/// not modify the resolved Edge fields.
///
/// Templated on the edge type so the helper can live at file scope without
/// naming NIImporter_OpenStreetMap::Edge (which is protected).
template <typename EdgeT>
void checkLaneCountBalance(const EdgeT* e) {
    int total = -1, forward = -1, backward = -1, bothWays = 0;
    std::string totalSrc, forwardSrc, backwardSrc, bothWaysSrc;
    const bool hasTotal    = pickHighestConfidence(e->evidence.lanesTotal,    total,    totalSrc);
    const bool hasForward  = pickHighestConfidence(e->evidence.lanesForward,  forward,  forwardSrc);
    const bool hasBackward = pickHighestConfidence(e->evidence.lanesBackward, backward, backwardSrc);
    const bool hasBothWays = pickHighestConfidence(e->evidence.lanesBothWays, bothWays, bothWaysSrc);

    // Need a total and at least one direction (or both_ways) to detect imbalance.
    if (!hasTotal || (!hasForward && !hasBackward && !hasBothWays)) {
        return;
    }
    // Need both directions or directions + both_ways to compute a sum.
    if (!hasForward || !hasBackward) {
        return;
    }

    const int sum = forward + backward + (hasBothWays ? bothWays : 0);
    if (sum != total) {
        std::ostringstream msg;
        msg << "Lane-count witnesses inconsistent for edge '" << e->id
            << "': total=" << total << " (from " << totalSrc << ")"
            << " but forward+backward" << (hasBothWays ? "+both_ways" : "") << "=" << sum
            << " (forward=" << forward << " from " << forwardSrc
            << ", backward=" << backward << " from " << backwardSrc;
        if (hasBothWays) {
            msg << ", both_ways=" << bothWays << " from " << bothWaysSrc;
        }
        msg << "). Resolved Edge fields unchanged.";
        WRITE_WARNING(msg.str());
    }
}

/// Two values are considered "in agreement" if they're equal. For doubles we
/// allow a small tolerance to avoid float-equality brittleness on values that
/// pass through unit conversion (e.g. 50 km/h vs 13.8889 m/s after rounding).
inline bool witnessValuesAgree(int a, int b) {
    return a == b;
}
inline bool witnessValuesAgree(double a, double b) {
    return std::abs(a - b) <= 0.01; // 1 cm/s for speed; trivial for widths
}

/// Witness-agreement check for any value type. When the list has at least
/// two witnesses that disagree, emit a warning naming every distinct value
/// and its source tag. Pure observation.
template <typename T, typename EdgeT>
void checkWitnessAgreement(const std::vector<NIOSMEvidence<T>>& witnesses,
                           const std::string& attrName,
                           const EdgeT* e) {
    if (witnesses.size() < 2) {
        return;
    }
    const T& firstValue = witnesses.front().value;
    bool allAgree = true;
    for (const auto& w : witnesses) {
        if (!witnessValuesAgree(w.value, firstValue)) {
            allAgree = false;
            break;
        }
    }
    if (allAgree) {
        return;
    }
    std::ostringstream msg;
    msg << "Disagreeing " << attrName << " witnesses for edge '" << e->id << "':";
    for (const auto& w : witnesses) {
        msg << " " << w.value << " (" << w.sourceTag << ")";
    }
    msg << ". Resolved Edge fields unchanged.";
    WRITE_WARNING(msg.str());
}

/// Lane-count plausibility warnings. Surfaces likely OSM tagging errors that
/// today are silently accepted: huge lane counts on small road types, or a
/// single lane on a motorway. Pure observation.
template <typename EdgeT>
void checkLaneCountPlausibility(const EdgeT* e) {
    int total = -1;
    std::string totalSrc;
    if (!pickHighestConfidence(e->evidence.lanesTotal, total, totalSrc)) {
        return; // no lane-count witness — nothing to flag
    }
    const std::string& type = e->myHighWayType; // possibly compound (a|b)

    auto hasType = [&type](const std::string& needle) {
        return type.find(needle) != std::string::npos;
    };

    if (total > 10) {
        WRITE_WARNINGF(TL("Implausible lane count for edge '%': lanes=% (from %, type=%). Real-world maximum is rarely above 10; likely a tagging error."),
                       toString(e->id), total, totalSrc, type);
    }
    if (total == 1 && hasType("highway.motorway")) {
        WRITE_WARNINGF(TL("Suspicious lane count for edge '%': lanes=1 on a motorway-class way (type=%, from %). Almost always a tagging error."),
                       toString(e->id), type, totalSrc);
    }
    if (total >= 4 && hasType("highway.residential")) {
        WRITE_WARNINGF(TL("Suspicious lane count for edge '%': lanes=% on highway=residential (from %). Likely should be classified as tertiary or higher."),
                       toString(e->id), total, totalSrc);
    }
    if (total > 6
            && !hasType("highway.motorway")
            && !hasType("highway.trunk")
            && !hasType("highway.primary")) {
        WRITE_WARNINGF(TL("Suspicious lane count for edge '%': lanes=% on type=% (from %). Counts above 6 are unusual outside motorway/trunk/primary."),
                       toString(e->id), total, type, totalSrc);
    }
}

/// Returns the maximum value across a witness list, ignoring missing
/// (empty) lists. Used when several lower-confidence witnesses point at the
/// same attribute and we want the most generous estimate.
int maxWitnessedValue(const std::vector<NIOSMEvidence<int>>& witnesses,
                      std::string& outSource) {
    int best = -1;
    for (const auto& w : witnesses) {
        if (w.value > best) {
            best = w.value;
            outSource = w.sourceTag;
        }
    }
    return best;
}

/// Lane-count inference. When the way has no explicit `lanes=` tag but
/// does have per-direction or per-lane pipe-count witnesses, derive a
/// total lane count from them and apply it to the Edge. Sets myNoLanes
/// (and myNoLanes{Forward,Backward}Explicit when only one direction is
/// witnessed) so the existing direction-split logic at insertEdge picks
/// up the value.
///
/// Skipped silently when --osm.repair is at warn level or below; only
/// 'infer' and 'aggressive' enable the actual mutation.
template <typename EdgeT>
bool repairLaneCountFromWitnesses(EdgeT* e) {
    if (e->myNoLanes >= 0) {
        return false; // explicit lanes= tag already present
    }
    std::string fwdSrc, bwdSrc, bwSrc;
    const int forward  = maxWitnessedValue(e->evidence.lanesForward,  fwdSrc);
    const int backward = maxWitnessedValue(e->evidence.lanesBackward, bwdSrc);
    const int bothWays = maxWitnessedValue(e->evidence.lanesBothWays, bwSrc);
    if (forward < 0 && backward < 0 && bothWays < 0) {
        return false; // no pipe-count witnesses either
    }

    // Derive the new value via the lane-count-balance conservation:
    // total = forward + backward + both_ways. Any missing direction
    // contributes 0.
    int newTotal = (forward >= 0 ? forward : 0)
                 + (backward >= 0 ? backward : 0)
                 + (bothWays >= 0 ? bothWays : 0);
    std::ostringstream provenance;
    bool first = true;
    auto addPart = [&](int v, const std::string& src, const char* name) {
        if (v < 0) return;
        if (!first) provenance << " + ";
        provenance << name << "=" << v << " (" << src << ")";
        first = false;
    };
    addPart(forward, fwdSrc, "lanes:forward witness");
    addPart(backward, bwdSrc, "lanes:backward witness");
    addPart(bothWays, bwSrc, "lanes:both_ways witness");

    e->myNoLanes = newTotal;
    if (forward >= 0 && backward < 0) {
        e->myNoLanesForwardExplicit = forward;
    } else if (backward >= 0 && forward < 0) {
        e->myNoLanesBackwardExplicit = backward;
    }
    WRITE_MESSAGEF(TL("Inferred lane count for edge '%': lanes=% (from %)."),
                   toString(e->id), toString(newTotal), provenance.str());
    return true;
}

/// Width plausibility warnings. Implausibly narrow per-lane width usually
/// means the way's lane count is wrong; implausibly wide usually means the
/// lane count is undercounted. Roundabouts and service ways have weird
/// geometry by design and are excluded from the upper bound.
template <typename EdgeT>
void checkWidthPlausibility(const EdgeT* e) {
    if (e->myWidth <= 0) {
        return;
    }
    int lanes = -1;
    std::string lanesSrc;
    if (!pickHighestConfidence(e->evidence.lanesTotal, lanes, lanesSrc)) {
        return;
    }
    if (lanes <= 0) {
        return;
    }
    const double perLane = e->myWidth / lanes;
    const std::string& type = e->myHighWayType;
    auto hasType = [&type](const std::string& needle) {
        return type.find(needle) != std::string::npos;
    };

    if (perLane < 2.0) {
        WRITE_WARNINGF(TL("Implausibly narrow per-lane width for edge '%': width=%/lanes=% = % m. Likely the lane count is over-stated."),
                       toString(e->id), toString(e->myWidth), toString(lanes), toString(perLane));
    }
    if (perLane > 6.0 && !e->myAmInRoundabout && !hasType("highway.service")) {
        WRITE_WARNINGF(TL("Implausibly wide per-lane width for edge '%': width=%/lanes=% = % m. Likely the lane count is under-counted."),
                       toString(e->id), toString(e->myWidth), toString(lanes), toString(perLane));
    }
}

/// Construction-class data-quality warnings. (a) highway=construction with
/// no construction=* tag means we cannot resolve the underlying class;
/// (b) start_date in the past on a still-construction-tagged way is stale
/// OSM data; (c) end_date set on a non-lifecycle-tagged way means the
/// mapper meant the road is gone but didn't say so.
template <typename EdgeT>
void checkLifecycleDataQuality(const EdgeT* e, const std::string& todayDate) {
    // (a) highway=construction with no construction=* tag.
    if (e->myHighWayType.find("highway.construction") != std::string::npos
            && e->myExtraTags.count("construction") == 0
            && e->myLifecycleStatus.empty()) {
        WRITE_WARNINGF(TL("Edge '%' is tagged highway=construction but no construction=* tag resolves the underlying class; using highway.construction defaults."),
                       toString(e->id));
    }
    // (b) construction-status way with start_date in the past relative to today.
    if (!e->myLifecycleStatus.empty() && e->myLifecycleStatus == "construction"
            && !e->myStartDate.empty() && !todayDate.empty()
            && isLikelyIsoDate(e->myStartDate)
            && padIsoDateLatest(e->myStartDate) < todayDate) {
        WRITE_WARNINGF(TL("Edge '%' is tagged construction:* but its start_date='%' is in the past relative to today (%); the OSM data may be stale (road may have opened)."),
                       toString(e->id), e->myStartDate, todayDate);
    }
    // (c) end_date set on an operational (non-lifecycle) way.
    if (e->myLifecycleStatus.empty() && !e->myEndDate.empty()) {
        WRITE_WARNINGF(TL("Edge '%' has end_date='%' but no lifecycle prefix (disused:, abandoned:, was:, etc.). The way is being treated as operational; the mapper may have meant to mark it as historical."),
                       toString(e->id), e->myEndDate);
    }
}

/// Maxspeed plausibility warnings, mirroring checkLaneCountPlausibility.
/// Speeds in the witness store are already in m/s as parsed by interpretSpeed.
/// Pure observation. Skips ways with no speed witness (typemap default would
/// dominate at resolve time, not visible here).
template <typename EdgeT>
void checkSpeedPlausibility(const EdgeT* e) {
    double speed = -1.0;
    std::string speedSrc;
    if (!pickHighestConfidence(e->evidence.speedForward, speed, speedSrc)) {
        return;
    }
    if (speed <= 0.0) {
        return; // signals/sign/MAXSPEED_UNGIVEN paths leave a non-positive value
    }
    const double kmh = speed * 3.6;
    const std::string& type = e->myHighWayType;
    auto hasType = [&type](const std::string& needle) {
        return type.find(needle) != std::string::npos;
    };

    if (kmh >= 100.0 && hasType("highway.residential")) {
        WRITE_WARNINGF(TL("Suspicious maxspeed for edge '%': % km/h on highway=residential (from %). Class probably wrong (likely tertiary or higher)."),
                       toString(e->id), toString(kmh), speedSrc);
    }
    if (kmh < 20.0 && hasType("highway.motorway")) {
        WRITE_WARNINGF(TL("Suspicious maxspeed for edge '%': % km/h on a motorway-class way (type=%, from %). Class or speed probably wrong."),
                       toString(e->id), toString(kmh), type, speedSrc);
    }
    if (kmh > 200.0) {
        WRITE_WARNINGF(TL("Implausible maxspeed for edge '%': % km/h (from %, type=%). Real-world legal maximums rarely exceed 200 km/h."),
                       toString(e->id), toString(kmh), speedSrc, type);
    }
}

/// Oneway-blocks-backward observer. If the resolved oneway state is "true"
/// (the way carries traffic in only one direction) and any lanesBackward
/// witnesses exist, then either:
///   - a per-mode oneway exception (oneway:bus=no etc.) explains the
///     contraflow lane: no warning, the contraflow case is handled, OR
///   - no exception is present: warn — this is the bare contradiction case
///     (oneway tag says no backward travel, yet a backward lane is tagged).
template <typename EdgeT>
void checkOnewayBackwardConflict(const EdgeT* e) {
    std::string onewayValue, onewaySrc;
    if (!pickHighestConfidence(e->evidence.oneway, onewayValue, onewaySrc)) {
        return; // no oneway witness, nothing to check
    }
    if (!isOnewayValue(onewayValue)) {
        return; // oneway resolves to bidirectional, no conflict possible
    }
    if (e->evidence.lanesBackward.empty()) {
        return; // no backward lane witnesses, no conflict
    }
    if (!e->evidence.onewayExceptions.empty()) {
        return; // contraflow case: per-mode exception explains backward lane(s)
    }

    std::ostringstream msg;
    msg << "Oneway/backward conflict for edge '" << e->id
        << "': oneway=" << onewayValue << " (from " << onewaySrc
        << ") but backward lane witnesses exist:";
    for (const auto& w : e->evidence.lanesBackward) {
        msg << " " << w.value << " (" << w.sourceTag << ")";
    }
    msg << ". No per-mode oneway:*=no exception was tagged. Resolved Edge fields unchanged.";
    WRITE_WARNING(msg.str());
}

} // anonymous namespace


// ---------------------------------------------------------------------------
// definitions of NIImporter_OpenStreetMap::EdgesHandler-methods
// ---------------------------------------------------------------------------
NIImporter_OpenStreetMap::EdgesHandler::EdgesHandler(
    const std::map<long long int, NIOSMNode*>& osmNodes,
    std::map<long long int, Edge*>& toFill, std::map<long long int, Edge*>& platformShapes,
    const NBTypeCont& tc):
    SUMOSAXHandler("osm - file"),
    myOSMNodes(osmNodes),
    myEdgeMap(toFill),
    myPlatformShapesMap(platformShapes),
    myTypeCont(tc) {

    const double unlimitedSpeed = OptionsCont::getOptions().getFloat("osm.speedlimit-none");

    mySpeedMap["nan"] = MAXSPEED_UNGIVEN;
    mySpeedMap["sign"] = MAXSPEED_UNGIVEN;
    mySpeedMap["signals"] = MAXSPEED_UNGIVEN;
    mySpeedMap["none"] = unlimitedSpeed;
    mySpeedMap["no"] = unlimitedSpeed;
    // OSM "walk" denotes pedestrian flow speed; the OSM wiki cites 1.0-1.4 m/s
    // typical walking, ~5 km/h here matches the upper end.
    // https://wiki.openstreetmap.org/wiki/Key:maxspeed#Special_values
    mySpeedMap["walk"] = 5. / 3.6;
    // https://wiki.openstreetmap.org/wiki/Key:source:maxspeed#Commonly_used_values
    mySpeedMap["AT:urban"] = 50. / 3.6;
    mySpeedMap["AT:rural"] = 100. / 3.6;
    mySpeedMap["AT:trunk"] = 100. / 3.6;
    mySpeedMap["AT:motorway"] = 130. / 3.6;
    mySpeedMap["AU:urban"] = 50. / 3.6;
    mySpeedMap["BE:urban"] = 50. / 3.6;
    mySpeedMap["BE:zone"] = 30. / 3.6;
    mySpeedMap["BE:motorway"] = 120. / 3.6;
    mySpeedMap["BE:zone30"] = 30. / 3.6;
    mySpeedMap["BE-VLG:rural"] = 70. / 3.6;
    mySpeedMap["BE-WAL:rural"] = 90. / 3.6;
    mySpeedMap["BE:school"] = 30. / 3.6;
    mySpeedMap["CZ:motorway"] = 130. / 3.6;
    mySpeedMap["CZ:trunk"] = 110. / 3.6;
    mySpeedMap["CZ:rural"] = 90. / 3.6;
    mySpeedMap["CZ:urban_motorway"] = 80. / 3.6;
    mySpeedMap["CZ:urban_trunk"] = 80. / 3.6;
    mySpeedMap["CZ:urban"] = 50. / 3.6;
    mySpeedMap["DE:motorway"] = unlimitedSpeed;
    mySpeedMap["DE:rural"] = 100. / 3.6;
    mySpeedMap["DE:urban"] = 50. / 3.6;
    mySpeedMap["DE:bicycle_road"] = 30. / 3.6;
    mySpeedMap["DK:motorway"] = 130. / 3.6;
    mySpeedMap["DK:rural"] = 80. / 3.6;
    mySpeedMap["DK:urban"] = 50. / 3.6;
    mySpeedMap["EE:urban"] = 50. / 3.6;
    mySpeedMap["EE:rural"] = 90. / 3.6;
    mySpeedMap["ES:urban"] = 50. / 3.6;
    mySpeedMap["ES:zone30"] = 30. / 3.6;
    mySpeedMap["FR:motorway"] = 130. / 3.6; // 110 (raining)
    mySpeedMap["FR:rural"] = 80. / 3.6;
    mySpeedMap["FR:urban"] = 50. / 3.6;
    mySpeedMap["FR:zone30"] = 30. / 3.6;
    mySpeedMap["HU:living_street"] = 20. / 3.6;
    mySpeedMap["HU:motorway"] = 130. / 3.6;
    mySpeedMap["HU:rural"] = 90. / 3.6;
    mySpeedMap["HU:trunk"] = 110. / 3.6;
    mySpeedMap["HU:urban"] = 50. / 3.6;
    mySpeedMap["IT:rural"] = 90. / 3.6;
    mySpeedMap["IT:motorway"] = 130. / 3.6;
    mySpeedMap["IT:urban"] = 50. / 3.6;
    mySpeedMap["JP:nsl"] = 60. / 3.6;
    mySpeedMap["JP:express"] = 100. / 3.6;
    mySpeedMap["LT:rural"] = 90. / 3.6;
    mySpeedMap["LT:urban"] = 50. / 3.6;
    mySpeedMap["NO:rural"] = 80. / 3.6;
    mySpeedMap["NO:urban"] = 50. / 3.6;
    mySpeedMap["ON:urban"] = 50. / 3.6;
    mySpeedMap["ON:rural"] = 80. / 3.6;
    mySpeedMap["PT:motorway"] = 120. / 3.6;
    mySpeedMap["PT:rural"] = 90. / 3.6;
    mySpeedMap["PT:trunk"] = 100. / 3.6;
    mySpeedMap["PT:urban"] = 50. / 3.6;
    mySpeedMap["RO:motorway"] = 130. / 3.6;
    mySpeedMap["RO:rural"] = 90. / 3.6;
    mySpeedMap["RO:trunk"] = 100. / 3.6;
    mySpeedMap["RO:urban"] = 50. / 3.6;
    mySpeedMap["RS:living_street"] = 30. / 3.6;
    mySpeedMap["RS:motorway"] = 130. / 3.6;
    mySpeedMap["RS:rural"] = 80. / 3.6;
    mySpeedMap["RS:trunk"] = 100. / 3.6;
    mySpeedMap["RS:urban"] = 50. / 3.6;
    mySpeedMap["RU:living_street"] = 20. / 3.6;
    mySpeedMap["RU:urban"] = 60. / 3.6;
    mySpeedMap["RU:rural"] = 90. / 3.6;
    mySpeedMap["RU:motorway"] = 110. / 3.6;
    const double seventy = StringUtils::parseSpeed("70mph");
    const double sixty = StringUtils::parseSpeed("60mph");
    const double thirtyMph = StringUtils::parseSpeed("30mph");
    const double twentyMph = StringUtils::parseSpeed("20mph");
    const double fiftyFiveMph = StringUtils::parseSpeed("55mph");
    const double sixtyFiveMph = StringUtils::parseSpeed("65mph");
    mySpeedMap["GB:motorway"] = seventy;
    mySpeedMap["GB:nsl_dual"] = seventy;
    mySpeedMap["GB:nsl_single"] = sixty;
    mySpeedMap["GB:urban"] = thirtyMph;
    mySpeedMap["UK:motorway"] = seventy;
    mySpeedMap["UK:nsl_dual"] = seventy;
    mySpeedMap["UK:nsl_single"] = sixty;
    mySpeedMap["UK:urban"] = thirtyMph;
    mySpeedMap["CH:urban"] = 50. / 3.6;
    mySpeedMap["CH:rural"] = 80. / 3.6;
    mySpeedMap["CH:trunk"] = 100. / 3.6;
    mySpeedMap["CH:motorway"] = 120. / 3.6;
    mySpeedMap["NL:urban"] = 50. / 3.6;
    mySpeedMap["NL:rural"] = 80. / 3.6;
    mySpeedMap["NL:trunk"] = 100. / 3.6;
    mySpeedMap["NL:motorway"] = 100. / 3.6;
    mySpeedMap["PL:urban"] = 50. / 3.6;
    mySpeedMap["PL:rural"] = 90. / 3.6;
    mySpeedMap["PL:trunk"] = 100. / 3.6;
    mySpeedMap["PL:expressway"] = 120. / 3.6;
    mySpeedMap["PL:motorway"] = 140. / 3.6;
    mySpeedMap["SE:urban"] = 50. / 3.6;
    mySpeedMap["SE:rural"] = 70. / 3.6;
    mySpeedMap["SE:trunk"] = 90. / 3.6;
    mySpeedMap["SE:motorway"] = 110. / 3.6;
    mySpeedMap["US:urban"] = thirtyMph;
    mySpeedMap["US:rural"] = fiftyFiveMph;
    mySpeedMap["US:motorway"] = sixtyFiveMph;
    mySpeedMap["UZ:living_street"] = 30. / 3.6;
    mySpeedMap["UZ:urban"] = 70. / 3.6;
    mySpeedMap["UZ:rural"] = 100. / 3.6;
    mySpeedMap["UZ:motorway"] = 110. / 3.6;
}

NIImporter_OpenStreetMap::EdgesHandler::~EdgesHandler() = default;

void
NIImporter_OpenStreetMap::EdgesHandler::myStartElement(int element, const SUMOSAXAttributes& attrs) {
    if (element == SUMO_TAG_WAY) {
        bool ok = true;
        const long long int id = attrs.get<long long int>(SUMO_ATTR_ID, nullptr, ok);
        const std::string& action = attrs.getOpt<std::string>(SUMO_ATTR_ACTION, nullptr, ok);
        if (action == "delete" || !ok) {
            myCurrentEdge = nullptr;
            return;
        }
        myCurrentEdge = new Edge(id);
    }
    // parse "nd" (node) elements
    if (element == SUMO_TAG_ND && myCurrentEdge != nullptr) {
        bool ok = true;
        long long int ref = attrs.get<long long int>(SUMO_ATTR_REF, nullptr, ok);
        if (ok) {
            auto node = myOSMNodes.find(ref);
            if (node == myOSMNodes.end()) {
                WRITE_WARNINGF(TL("The referenced geometry information (ref='%') is not known"), toString(ref));
                return;
            }

            ref = node->second->id; // node may have been substituted
            if (myCurrentEdge->myCurrentNodes.empty() ||
                    myCurrentEdge->myCurrentNodes.back() != ref) { // avoid consecutive duplicates
                myCurrentEdge->myCurrentNodes.push_back(ref);
            }

        }
    }
    if (element == SUMO_TAG_TAG && myCurrentEdge != nullptr) {
        bool ok = true;
        std::string key = attrs.get<std::string>(SUMO_ATTR_K, toString(myCurrentEdge->id).c_str(), ok, false);
        // Preserve the raw OSM key for --osm.all-attributes retention so
        // lifecycle-prefixed forms like "construction:highway" survive in
        // the <param> output even though we strip the prefix for the rest
        // of the parser below.
        const std::string rawKey = key;
        // Lifecycle prefix handling. Tags like construction:highway=residential
        // describe ways that are not currently operational but use the
        // attribute set of the named class. Strip the prefix, record the
        // status on the edge, and let the rest of the parser process the
        // tags as if they applied to the underlying class. Edges with a
        // lifecycle status are discarded at myEndElement by default.
        {
            static const char* const lifecyclePrefixes[] = {
                "construction:", "proposed:", "disused:", "abandoned:",
                "razed:", "demolished:", "removed:", "was:", "planned:"
            };
            for (const char* const prefix : lifecyclePrefixes) {
                const std::string p = prefix;
                if (StringUtils::startsWith(key, p)) {
                    if (myCurrentEdge->myLifecycleStatus.empty()) {
                        myCurrentEdge->myLifecycleStatus = p.substr(0, p.size() - 1);
                    }
                    key = key.substr(p.size());
                    break;
                }
            }
        }
        if (key.size() > 6 && StringUtils::startsWith(key, "busway:")) {
            // handle special busway keys
            const std::string buswaySpec = key.substr(7);
            key = "busway";
            if (buswaySpec == "right") {
                myCurrentEdge->myBuswayType = (WayType)(myCurrentEdge->myBuswayType | WAY_FORWARD);
            } else if (buswaySpec == "left") {
                myCurrentEdge->myBuswayType = (WayType)(myCurrentEdge->myBuswayType | WAY_BACKWARD);
            } else if (buswaySpec == "both") {
                myCurrentEdge->myBuswayType = (WayType)(myCurrentEdge->myBuswayType | WAY_BOTH);
            } else {
                key = "ignore";
            }
        }
        if (myAllAttributes && (myExtraAttributes.count(rawKey) != 0 || myExtraAttributes.size() == 0)) {
            const std::string info = "way=" + toString(myCurrentEdge->id) + ", k=" + rawKey;
            myCurrentEdge->setParameter(rawKey, attrs.get<std::string>(SUMO_ATTR_V, info.c_str(), ok, false));
        }
        // we check whether the key is relevant (and we really need to transcode the value) to avoid hitting #1636
        if (!StringUtils::endsWith(key, "way")
                && !StringUtils::startsWith(key, "lanes")
                && key != "maxspeed" && key != "maxspeed:type"
                && key != "zone:maxspeed"
                && key != "maxspeed:forward" && key != "maxspeed:backward"
                && key != "junction" && key != "name" && key != "tracks" && key != "layer"
                && key != "route"
                && !StringUtils::startsWith(key, "cycleway")
                && !StringUtils::startsWith(key, "sidewalk")
                && key != "ref"
                && key != "highspeed"
                && !StringUtils::startsWith(key, "parking")
                && !StringUtils::startsWith(key, "change")
                && !StringUtils::startsWith(key, "vehicle:lanes")
                && key != "postal_code"
                && key != "railway:preferred_direction"
                && key != "railway:bidirectional"
                && key != "railway:track_ref"
                && key != "usage"
                && key != "access"
                && key != "emergency"
                && key != "service"
                && key != "electrified"
                && key != "segregated"
                && key != "bus"
                && key != "psv"
                && key != "foot"
                && key != "bicycle"
                && key != "vehicle"
                && key != "motor_vehicle"
                && key != "motorcar"
                && key != "hgv"
                && key != "taxi"
                && key != "motorcycle"
                && key != "moped"
                && key != "start_date"
                && key != "end_date"
                && key != "oneway:bicycle"
                && key != "oneway:bus"
                && key != "oneway:psv"
                && key != "oneway:hgv"
                && key != "oneway:motor_vehicle"
                && key != "oneway:motorcar"
                && key != "access:conditional"
                && key != "motor_vehicle:conditional"
                && key != "vehicle:conditional"
                && key != "bus:conditional"
                && key != "hgv:conditional"
                && key != "maxspeed:conditional"
                && key != "maxspeed:variable"
                && key != "maxspeed:advisory"
                && key != "maxspeed:lanes"
                && key != "maxspeed:lanes:forward"
                && key != "maxspeed:lanes:backward"
                && key != "maxspeed:hgv"
                && key != "maxspeed:bus"
                && key != "maxspeed:bicycle"
                && key != "maxspeed:taxi"
                && key != "maxspeed:motorcycle"
                && key != "maxspeed:moped"
                && key != "placement"
                && key != "bus:lanes"
                && key != "bus:lanes:forward"
                && key != "bus:lanes:backward"
                && key != "lanes:bus"
                && key != "lanes:bus:forward"
                && key != "lanes:bus:backward"
                && key != "psv:lanes"
                && key != "psv:lanes:forward"
                && key != "psv:lanes:backward"
                && key != "lanes:psv"
                && key != "lanes:psv:forward"
                && key != "lanes:psv:backward"
                && key != "hgv:lanes"
                && key != "hgv:lanes:forward"
                && key != "hgv:lanes:backward"
                && key != "lanes:hgv"
                && key != "lanes:hgv:forward"
                && key != "lanes:hgv:backward"
                && key != "taxi:lanes"
                && key != "taxi:lanes:forward"
                && key != "taxi:lanes:backward"
                && key != "lanes:taxi"
                && key != "lanes:taxi:forward"
                && key != "lanes:taxi:backward"
                && key != "motorcycle:lanes"
                && key != "motorcycle:lanes:forward"
                && key != "motorcycle:lanes:backward"
                && key != "moped:lanes"
                && key != "moped:lanes:forward"
                && key != "moped:lanes:backward"
                && key != "motor_vehicle:lanes"
                && key != "motor_vehicle:lanes:forward"
                && key != "motor_vehicle:lanes:backward"
                && key != "motorcar:lanes"
                && key != "motorcar:lanes:forward"
                && key != "motorcar:lanes:backward"
                && key != "bicycle:lanes"
                && key != "bicycle:lanes:forward"
                && key != "bicycle:lanes:backward"
                && !StringUtils::startsWith(key, "width")
                && !(StringUtils::startsWith(key, "turn:") && key.find(":lanes") != std::string::npos)
                && key != "public_transport") {
            return;
        }
        const std::string value = attrs.get<std::string>(SUMO_ATTR_V, toString(myCurrentEdge->id).c_str(), ok, false);

        if (key == "highway" || key == "railway" || key == "waterway" || StringUtils::startsWith(key, "cycleway")
                || key == "busway" || key == "route" || StringUtils::startsWith(key, "sidewalk") || key == "highspeed"
                || key == "aeroway" || key == "aerialway" || key == "usage" || key == "service") {
            // build type id
            if (key != "highway" || myTypeCont.knows(key + "." + value)) {
                myCurrentEdge->myCurrentIsRoad = true;
            }
            // special cycleway stuff https://wiki.openstreetmap.org/wiki/Key:cycleway
            if (key == "cycleway") {
                if (value == "no" || value == "none" || value == "separate") {
                    myCurrentEdge->myCyclewayType = WAY_NONE;
                } else if (value == "both") {
                    myCurrentEdge->myCyclewayType = WAY_BOTH;
                } else if (value == "right") {
                    myCurrentEdge->myCyclewayType = WAY_FORWARD;
                } else if (value == "left") {
                    myCurrentEdge->myCyclewayType = WAY_BACKWARD;
                } else if (value == "opposite_track") {
                    myCurrentEdge->myCyclewayType = WAY_BACKWARD;
                } else if (value == "opposite_lane") {
                    myCurrentEdge->myCyclewayType = WAY_BACKWARD;
                } else if (value == "opposite") {
                    // according to the wiki ref above, this should rather be a bidi lane, see #13438
                    myCurrentEdge->myCyclewayType = WAY_BACKWARD;
                }
            }
            if (key == "cycleway:left") {
                if (myCurrentEdge->myCyclewayType == WAY_UNKNOWN) {
                    myCurrentEdge->myCyclewayType = WAY_NONE;
                }
                if (value == "yes" || value == "lane" || value == "track") {
                    myCurrentEdge->myCyclewayType = (WayType)(myCurrentEdge->myCyclewayType | WAY_BACKWARD);
                }
                key = "cycleway"; // for type adaption
            }
            if (key == "cycleway:right") {
                if (myCurrentEdge->myCyclewayType == WAY_UNKNOWN) {
                    myCurrentEdge->myCyclewayType = WAY_NONE;
                }
                if (value == "yes" || value == "lane" || value == "track") {
                    myCurrentEdge->myCyclewayType = (WayType)(myCurrentEdge->myCyclewayType | WAY_FORWARD);
                }
                key = "cycleway"; // for type adaption
            }
            if (key == "cycleway:both") {
                if (myCurrentEdge->myCyclewayType == WAY_UNKNOWN) {
                    if (value == "no" || value == "none" || value == "separate") {
                        myCurrentEdge->myCyclewayType = WAY_NONE;
                    }
                    if (value == "yes" || value == "lane" || value == "track") {
                        myCurrentEdge->myCyclewayType = WAY_BOTH;
                    }
                }
                key = "cycleway"; // for type adaption
            }
            if (key == "cycleway" && value != "lane" && value != "track" && value != "opposite_track" && value != "opposite_lane") {
                // typemap covers only the lane and track cases
                return;
            }
            if (StringUtils::startsWith(key, "cycleway:")) {
                // no need to extend the type id for other cycleway sub tags
                return;
            }
            // special sidewalk stuff
            if (key == "sidewalk") {
                if (value == "no" || value == "none" || value == "separate") {
                    myCurrentEdge->mySidewalkType = WAY_NONE;
                    if (value == "separate") {
                        myCurrentEdge->myExtraDisallowed |= SVC_PEDESTRIAN;
                    }
                } else if (value == "both" || value == "yes") {
                    myCurrentEdge->mySidewalkType = WAY_BOTH;
                } else if (value == "right") {
                    myCurrentEdge->mySidewalkType = WAY_FORWARD;
                } else if (value == "left") {
                    myCurrentEdge->mySidewalkType = WAY_BACKWARD;
                }
            }
            if (key == "sidewalk:left") {
                if (myCurrentEdge->mySidewalkType == WAY_UNKNOWN) {
                    myCurrentEdge->mySidewalkType = WAY_NONE;
                }
                if (value == "yes") {
                    myCurrentEdge->mySidewalkType = (WayType)(myCurrentEdge->mySidewalkType | WAY_BACKWARD);
                }
                if (value == "separate") {
                    myCurrentEdge->myExtraDisallowed |= SVC_PEDESTRIAN;
                }
            }
            if (key == "sidewalk:right") {
                if (myCurrentEdge->mySidewalkType == WAY_UNKNOWN) {
                    myCurrentEdge->mySidewalkType = WAY_NONE;
                }
                if (value == "yes") {
                    myCurrentEdge->mySidewalkType = (WayType)(myCurrentEdge->mySidewalkType | WAY_FORWARD);
                }
                if (value == "separate") {
                    myCurrentEdge->myExtraDisallowed |= SVC_PEDESTRIAN;
                }
            }
            if (key == "sidewalk:both") {
                if (myCurrentEdge->mySidewalkType == WAY_UNKNOWN) {
                    if (value == "no" || value == "none" || value == "separate") {
                        myCurrentEdge->mySidewalkType = WAY_NONE;
                        if (value == "separate") {
                            myCurrentEdge->myExtraDisallowed |= SVC_PEDESTRIAN;
                        }
                    }
                    if (value == "yes") {
                        myCurrentEdge->mySidewalkType = WAY_BOTH;
                    }
                }
            }
            if (StringUtils::startsWith(key, "sidewalk")) {
                // no need to extend the type id
                return;
            }
            // special busway stuff
            if (key == "busway") {
                if (value == "no") {
                    return;
                }
                if (value == "opposite_track") {
                    myCurrentEdge->myBuswayType = WAY_BACKWARD;
                } else if (value == "opposite_lane") {
                    myCurrentEdge->myBuswayType = WAY_BACKWARD;
                }
                // no need to extend the type id
                return;
            }
            std::string singleTypeID = key + "." + value;
            if (key == "highspeed") {
                if (value == "no") {
                    return;
                }
                singleTypeID = "railway.highspeed";
            }
            addType(singleTypeID);

        } else if (key == "bus" || key == "psv") {
            // OSM 'psv' = public service vehicle, covers buses *and* taxis
            // (including legal carve-outs on bus/psv lanes). 'bus' is
            // bus-only. Absence of either tag is not informative -- it
            // does not imply disallow.
            const SVCPermissions psvClasses = (key == "psv") ? (SVC_BUS | SVC_TAXI) : SVC_BUS;
            try {
                if (StringUtils::toBool(value)) {
                    myCurrentEdge->myExtraAllowed |= psvClasses;
                    // explicit per-mode allow: wins over broader implicit
                    // disallows like motor_vehicle=no
                    myCurrentEdge->myExplicitlyAllowed |= psvClasses;
                    addType(key);
                } else {
                    myCurrentEdge->myExtraDisallowed |= psvClasses;
                }
            } catch (const BoolFormatException&) {
                myCurrentEdge->myExtraAllowed |= psvClasses;
                myCurrentEdge->myExplicitlyAllowed |= psvClasses;
                addType(key);
            }
        } else if (key == "emergency") {
            try {
                if (StringUtils::toBool(value)) {
                    myCurrentEdge->myExtraAllowed |= SVC_AUTHORITY | SVC_EMERGENCY;
                    myCurrentEdge->myExplicitlyAllowed |= SVC_AUTHORITY | SVC_EMERGENCY;
                }
            } catch (const BoolFormatException&) {
                myCurrentEdge->myExtraAllowed |= SVC_AUTHORITY | SVC_EMERGENCY;
                myCurrentEdge->myExplicitlyAllowed |= SVC_AUTHORITY | SVC_EMERGENCY;
            }
        } else if (key == "access") {
            // OSM access semantics (https://wiki.openstreetmap.org/wiki/Key:access).
            // Strict: a broader prohibition disallows everything except the
            // emergency/authority real-world convention. Buses / public
            // transport are NOT implicitly exempt -- they require an explicit
            // bus=yes / psv=yes to be re-allowed (which works via the line-524
            // explicit-allow tracking).
            if (value == "no" || value == "private") {
                myCurrentEdge->myExtraDisallowed |= ~(SVC_EMERGENCY | SVC_AUTHORITY);
            } else if (value == "destination" || value == "customers" || value == "delivery") {
                // restricted to destination/delivery traffic
                myCurrentEdge->myExtraDisallowed |= ~(SVC_EMERGENCY | SVC_AUTHORITY | SVC_DELIVERY);
            } else if (value == "agricultural" || value == "forestry") {
                // No direct SUMO equivalent; restrict to the same minimum set as 'no'
                myCurrentEdge->myExtraDisallowed |= ~(SVC_EMERGENCY | SVC_AUTHORITY);
                WRITE_WARNINGF(TL("Edge '%' has access=% which has no direct SUMO equivalent; treating as restricted (only emergency/authority allowed)."),
                               toString(myCurrentEdge->id), value);
            }
            // access=yes and access=permissive are no-ops (default unrestricted).
        } else if (key == "vehicle") {
            // OSM "vehicle" covers all wheeled vehicles (motor + bicycle).
            // Strict: broader vehicle prohibitions disallow ALL wheeled
            // classes including public transport. Explicit per-mode tags
            // (bus=yes, etc.) re-allow specific classes via the line-524
            // explicit-allow tracking; emergency/authority are kept as a
            // real-world convention.
            if (value == "no" || value == "private") {
                myCurrentEdge->myExtraDisallowed |=
                    (SVC_ROAD_MOTOR_CLASSES | SVC_BICYCLE | SVC_SCOOTER)
                    & ~(SVC_EMERGENCY | SVC_AUTHORITY);
            } else if (value == "destination" || value == "customers" || value == "delivery") {
                myCurrentEdge->myExtraDisallowed |=
                    (SVC_ROAD_MOTOR_CLASSES | SVC_BICYCLE | SVC_SCOOTER)
                    & ~(SVC_EMERGENCY | SVC_AUTHORITY | SVC_DELIVERY);
            } else if (value == "yes" || value == "permissive") {
                myCurrentEdge->myExtraAllowed |= SVC_ROAD_MOTOR_CLASSES | SVC_BICYCLE | SVC_SCOOTER;
            }
        } else if (key == "motor_vehicle" || key == "motorcar") {
            // Block or permit motorised road traffic. Strict: broader
            // prohibition disallows ALL motorised classes including buses;
            // explicit motor_vehicle=yes / bus=yes re-allows via explicit-allow tracking.
            try {
                if (StringUtils::toBool(value)) {
                    myCurrentEdge->myExtraAllowed |= SVC_ROAD_MOTOR_CLASSES;
                    myCurrentEdge->myExplicitlyAllowed |= SVC_ROAD_MOTOR_CLASSES;
                } else {
                    myCurrentEdge->myExtraDisallowed |=
                        SVC_ROAD_MOTOR_CLASSES & ~(SVC_EMERGENCY | SVC_AUTHORITY);
                }
            } catch (const BoolFormatException&) {
                // destination/private/customers/etc. on motor_vehicle: treat
                // as restrictive like 'no', warning the user about the
                // unsupported nuance.
                myCurrentEdge->myExtraDisallowed |=
                    SVC_ROAD_MOTOR_CLASSES & ~(SVC_EMERGENCY | SVC_AUTHORITY);
                if (value != "designated") {
                    WRITE_WARNINGF(TL("Edge '%' has %=% ; treating as motor traffic disallowed."),
                                   toString(myCurrentEdge->id), key, value);
                }
            }
        } else if (key == "hgv") {
            try {
                if (StringUtils::toBool(value)) {
                    myCurrentEdge->myExtraAllowed |= SVC_TRUCK | SVC_TRAILER;
                    myCurrentEdge->myExplicitlyAllowed |= SVC_TRUCK | SVC_TRAILER;
                } else {
                    myCurrentEdge->myExtraDisallowed |= SVC_TRUCK | SVC_TRAILER;
                }
            } catch (const BoolFormatException&) {
                myCurrentEdge->myExtraAllowed |= SVC_TRUCK | SVC_TRAILER;
                myCurrentEdge->myExplicitlyAllowed |= SVC_TRUCK | SVC_TRAILER;
            }
        } else if (key == "taxi") {
            try {
                if (StringUtils::toBool(value)) {
                    myCurrentEdge->myExtraAllowed |= SVC_TAXI;
                    myCurrentEdge->myExplicitlyAllowed |= SVC_TAXI;
                } else {
                    myCurrentEdge->myExtraDisallowed |= SVC_TAXI;
                }
            } catch (const BoolFormatException&) {
                myCurrentEdge->myExtraAllowed |= SVC_TAXI;
                myCurrentEdge->myExplicitlyAllowed |= SVC_TAXI;
            }
        } else if (key == "motorcycle") {
            try {
                if (StringUtils::toBool(value)) {
                    myCurrentEdge->myExtraAllowed |= SVC_MOTORCYCLE;
                    myCurrentEdge->myExplicitlyAllowed |= SVC_MOTORCYCLE;
                } else {
                    myCurrentEdge->myExtraDisallowed |= SVC_MOTORCYCLE;
                }
            } catch (const BoolFormatException&) {
                myCurrentEdge->myExtraAllowed |= SVC_MOTORCYCLE;
                myCurrentEdge->myExplicitlyAllowed |= SVC_MOTORCYCLE;
            }
        } else if (key == "moped") {
            try {
                if (StringUtils::toBool(value)) {
                    myCurrentEdge->myExtraAllowed |= SVC_MOPED;
                    myCurrentEdge->myExplicitlyAllowed |= SVC_MOPED;
                } else {
                    myCurrentEdge->myExtraDisallowed |= SVC_MOPED;
                }
            } catch (const BoolFormatException&) {
                myCurrentEdge->myExtraAllowed |= SVC_MOPED;
                myCurrentEdge->myExplicitlyAllowed |= SVC_MOPED;
            }
        } else if (key == "start_date") {
            myCurrentEdge->myStartDate = value;
        } else if (key == "end_date") {
            myCurrentEdge->myEndDate = value;
        } else if (StringUtils::startsWith(key, "width:lanes")) {
            try {
                const std::vector<std::string> values = StringTokenizer(value, "|").getVector();
                std::vector<double> widthLanes;
                for (std::string width : values) {
                    const double parsedWidth = width == "" ? -1 : StringUtils::parseDist(width);
                    widthLanes.push_back(parsedWidth);
                }

                const bool reverseOneway = (myCurrentEdge->myIsOneWay == "-1"
                                            || myCurrentEdge->myIsOneWay == "reverse");
                if (key == "width:lanes:backward" || (key == "width:lanes" && reverseOneway)) {
                    myCurrentEdge->myWidthLanesBackward = widthLanes;
                    myCurrentEdge->evidence.lanesBackward.emplace_back(
                        (int)values.size(), key + " pipe count", NIOSMConfidence::MEDIUM_HIGH);
                } else if (key == "width:lanes:forward" || (key == "width:lanes" && !reverseOneway)) {
                    myCurrentEdge->myWidthLanesForward = widthLanes;
                    myCurrentEdge->evidence.lanesForward.emplace_back(
                        (int)values.size(), key + " pipe count", NIOSMConfidence::MEDIUM_HIGH);
                } else {
                    WRITE_WARNINGF(TL("Using default lane width for edge '%' as key '%' could not be parsed."), toString(myCurrentEdge->id), key);
                }
            } catch (const NumberFormatException&) {
                WRITE_WARNINGF(TL("Using default lane width for edge '%' as value '%' could not be parsed."), toString(myCurrentEdge->id), value);
            }
        } else if (key == "width") {
            try {
                myCurrentEdge->myWidth = StringUtils::parseDist(value);
                myCurrentEdge->evidence.width.emplace_back(
                    myCurrentEdge->myWidth, "width", NIOSMConfidence::HIGH);
            } catch (const NumberFormatException&) {
                WRITE_WARNINGF(TL("Using default width for edge '%' as value '%' could not be parsed."), toString(myCurrentEdge->id), value);
            }
        } else if (key == "foot") {
            if (value == "use_sidepath" || value == "no") {
                myCurrentEdge->myExtraDisallowed |= SVC_PEDESTRIAN;
            } else if (value == "yes" || value == "designated" || value == "permissive") {
                myCurrentEdge->myExtraAllowed |= SVC_PEDESTRIAN;
            }
        } else if (key == "bicycle") {
            if (value == "use_sidepath" || value == "no") {
                myCurrentEdge->myExtraDisallowed |= SVC_BICYCLE;
            } else if (value == "yes" || value == "designated" || value == "permissive") {
                myCurrentEdge->myExtraAllowed |= SVC_BICYCLE;
            }
        } else if (key == "oneway:bicycle") {
            myCurrentEdge->myExtraTags["oneway:bicycle"] = value;
        } else if (key == "oneway:bus" || key == "oneway:psv") {
            if (value == "no") {
                // need to add a bus way in reversed direction of way
                myCurrentEdge->myBuswayType = WAY_BACKWARD;
                myCurrentEdge->evidence.onewayExceptions.emplace_back(
                    key.substr(7), key, NIOSMConfidence::HIGH);
            }
        } else if (key == "oneway:hgv" || key == "oneway:motor_vehicle" || key == "oneway:motorcar") {
            // Per-mode oneway exceptions for HGVs and motor traffic.
            // Captured as a witness so the reconciler sees the contraflow
            // case (see checkOnewayBackwardConflict). No dedicated
            // SUMO-level reverse-access mechanism yet (unlike buses with
            // myBuswayType); record as parameter for downstream tooling.
            if (value == "no") {
                myCurrentEdge->evidence.onewayExceptions.emplace_back(
                    key.substr(7), key, NIOSMConfidence::HIGH);
                myCurrentEdge->setParameter(key, value);
            }
        } else if (key == "access:conditional"
                   || key == "motor_vehicle:conditional"
                   || key == "vehicle:conditional"
                   || key == "bus:conditional"
                   || key == "hgv:conditional"
                   || key == "maxspeed:conditional"
                   || key == "maxspeed:variable"
                   || key == "maxspeed:advisory") {
            // Surface conditional/variable/advisory constraints as
            // parameters on the edge. The value language ("60 @ (Mo-Fr
            // 07:00-19:00)", "peak_traffic", etc.) describes time-varying
            // behaviour that the static network can't express; the edge
            // keeps whatever non-conditional maxspeed (or typemap default)
            // it would otherwise have. Brief informational warning so the
            // user knows the conditional data is preserved as <param> but
            // not enforced at runtime.
            WRITE_WARNINGF(TL("Edge '%' has time-varying tag %=%; preserved as <param>, static edge speed unchanged."),
                           toString(myCurrentEdge->id), key, value);
            myCurrentEdge->setParameter(key, value);
        } else if (key == "placement") {
            if (!interpretPlacement(value, myCurrentEdge->myPlacement, myCurrentEdge->myPlacementLane)) {
                WRITE_WARNINGF(TL("Ignoring unsupported placement value '%' for edge '%'."), value, myCurrentEdge->id);
            }
        } else if (key == "lanes") {
            try {
                myCurrentEdge->myNoLanes = StringUtils::toInt(value);
                myCurrentEdge->evidence.lanesTotal.emplace_back(
                    myCurrentEdge->myNoLanes, "lanes", NIOSMConfidence::HIGH);
            } catch (NumberFormatException&) {
                // might be a list of values
                StringTokenizer st(value, ";", true);
                std::vector<std::string> list = st.getVector();
                if (list.size() >= 2) {
                    int minLanes = std::numeric_limits<int>::max();
                    try {
                        for (auto& i : list) {
                            const int numLanes = StringUtils::toInt(StringUtils::prune(i));
                            minLanes = MIN2(minLanes, numLanes);
                        }
                        myCurrentEdge->myNoLanes = minLanes;
                        myCurrentEdge->evidence.lanesTotal.emplace_back(
                            minLanes, "lanes (semicolon-list min)", NIOSMConfidence::MEDIUM);
                        WRITE_WARNINGF(TL("Edge '%' has lanes=% which is a non-standard semicolon list; using the minimum (%). The OSM lanes key expects a single integer; verify the source data."),
                                       toString(myCurrentEdge->id), value, toString(minLanes));
                    } catch (NumberFormatException&) {
                        WRITE_WARNINGF(TL("Value of key '%' is not numeric ('%') in edge '%'."), key, value, myCurrentEdge->id);
                    }
                }
            } catch (EmptyData&) {
                WRITE_WARNINGF(TL("Value of key '%' is not numeric ('%') in edge '%'."), key, value, myCurrentEdge->id);
            }
        } else if (key == "lanes:forward") {
            try {
                const int numLanes = StringUtils::toInt(value);
                const int bothWays = myCurrentEdge->evidence.lanesBothWays.empty() ? 0 : myCurrentEdge->evidence.lanesBothWays.back().value;
                if (myCurrentEdge->myNoLanesBackwardExplicit > 0 && myCurrentEdge->myNoLanes < 0) {
                    // fix lane count in case only lanes:forward and lanes:backward are set
                    myCurrentEdge->myNoLanes = numLanes + myCurrentEdge->myNoLanesBackwardExplicit + bothWays;
                }
                myCurrentEdge->myNoLanesForwardExplicit = numLanes;
                myCurrentEdge->evidence.lanesForward.emplace_back(
                    numLanes, "lanes:forward", NIOSMConfidence::HIGH);
            } catch (...) {
                WRITE_WARNINGF(TL("Value of key '%' is not numeric ('%') in edge '%'."), key, value, myCurrentEdge->id);
            }
        } else if (key == "lanes:backward") {
            try {
                const int numLanes = StringUtils::toInt(value);
                const int bothWays = myCurrentEdge->evidence.lanesBothWays.empty() ? 0 : myCurrentEdge->evidence.lanesBothWays.back().value;
                if (myCurrentEdge->myNoLanesForwardExplicit > 0 && myCurrentEdge->myNoLanes < 0) {
                    // fix lane count in case only lanes:forward and lanes:backward are set
                    myCurrentEdge->myNoLanes = numLanes + myCurrentEdge->myNoLanesForwardExplicit + bothWays;
                }
                myCurrentEdge->myNoLanesBackwardExplicit = numLanes;
                myCurrentEdge->evidence.lanesBackward.emplace_back(
                    numLanes, "lanes:backward", NIOSMConfidence::HIGH);
            } catch (...) {
                WRITE_WARNINGF(TL("Value of key '%' is not numeric ('%') in edge '%'."), key, value, myCurrentEdge->id);
            }
        } else if (key == "lanes:both_ways") {
            // OSM's center two-way left-turn lane (TWLTL). Parsed into the
            // witness store; downstream reconciliation can use it via the
            // lane-count balance constraint. SUMO has no native TWLTL
            // primitive yet, so the value is recorded as a parameter for
            // downstream consumers.
            try {
                const int numLanes = StringUtils::toInt(value);
                myCurrentEdge->evidence.lanesBothWays.emplace_back(
                    numLanes, "lanes:both_ways", NIOSMConfidence::HIGH);
                myCurrentEdge->setParameter("lanes:both_ways", value);
                if (myCurrentEdge->myNoLanesForwardExplicit > 0 && myCurrentEdge->myNoLanesBackwardExplicit > 0 && myCurrentEdge->myNoLanes < 0) {
                    myCurrentEdge->myNoLanes = myCurrentEdge->myNoLanesForwardExplicit + myCurrentEdge->myNoLanesBackwardExplicit + numLanes;
                }
            } catch (...) {
                WRITE_WARNINGF(TL("Value of key '%' is not numeric ('%') in edge '%'."), key, value, myCurrentEdge->id);
            }
        } else if (myCurrentEdge->myMaxSpeed == MAXSPEED_UNGIVEN &&
                   (key == "maxspeed" || key == "maxspeed:type" || key == "maxspeed:forward" || key == "zone:maxspeed")) {
            // both 'maxspeed' and 'maxspeed:type' may be given so we must take care not to overwrite an already seen value
            myCurrentEdge->myMaxSpeed = interpretSpeed(key, value);
            // maxspeed:type and zone:maxspeed are country/zone-defaulted lookups, not direct measurements
            const NIOSMConfidence speedConf = (key == "maxspeed:type" || key == "zone:maxspeed")
                ? NIOSMConfidence::MEDIUM_HIGH : NIOSMConfidence::HIGH;
            myCurrentEdge->evidence.speedForward.emplace_back(myCurrentEdge->myMaxSpeed, key, speedConf);
        } else if (key == "maxspeed:backward" && myCurrentEdge->myMaxSpeedBackward == MAXSPEED_UNGIVEN) {
            myCurrentEdge->myMaxSpeedBackward = interpretSpeed(key, value);
            myCurrentEdge->evidence.speedBackward.emplace_back(
                myCurrentEdge->myMaxSpeedBackward, "maxspeed:backward", NIOSMConfidence::HIGH);
        } else if (key == "maxspeed:lanes" || key == "maxspeed:lanes:forward"
                   || key == "maxspeed:lanes:backward") {
            // Per-lane maxspeed override. Pipe-separated, one entry per
            // lane. An empty entry leaves the lane at the edge default.
            const std::vector<std::string> values = StringTokenizer(value, "|").getVector();
            std::vector<double> perLane;
            perLane.reserve(values.size());
            for (const std::string& v : values) {
                if (v.empty()) {
                    perLane.push_back(MAXSPEED_UNGIVEN);
                } else {
                    try {
                        perLane.push_back(interpretSpeed(key, v));
                    } catch (...) {
                        WRITE_WARNINGF(TL("Edge '%' has unparseable maxspeed:lanes entry '%' in tag %=%; defaulting that lane."),
                                       toString(myCurrentEdge->id), v, key, value);
                        perLane.push_back(MAXSPEED_UNGIVEN);
                    }
                }
            }
            const bool reverseOneway = (myCurrentEdge->myIsOneWay == "-1"
                                        || myCurrentEdge->myIsOneWay == "reverse");
            if (key == "maxspeed:lanes:backward" || (key == "maxspeed:lanes" && reverseOneway)) {
                myCurrentEdge->mySpeedLanesBackward = perLane;
                myCurrentEdge->evidence.lanesBackward.emplace_back(
                    (int)perLane.size(), key + " pipe count", NIOSMConfidence::MEDIUM_HIGH);
            } else {
                myCurrentEdge->mySpeedLanesForward = perLane;
                myCurrentEdge->evidence.lanesForward.emplace_back(
                    (int)perLane.size(), key + " pipe count", NIOSMConfidence::MEDIUM_HIGH);
            }
        } else if (key == "maxspeed:hgv" || key == "maxspeed:bus"
                   || key == "maxspeed:bicycle" || key == "maxspeed:taxi"
                   || key == "maxspeed:motorcycle" || key == "maxspeed:moped") {
            // Per-vehicle-class maxspeed. SUMO has no per-class lane
            // speed in the network model; surface as <param> so downstream
            // tools (TraCI, custom routing) can act on it.
            myCurrentEdge->setParameter(key, value);
        } else if (key == "junction") {
            if ((value == "roundabout" || value == "circular") && myCurrentEdge->myIsOneWay.empty()) {
                myCurrentEdge->myIsOneWay = "yes";
            }
            if (value == "roundabout" || value == "circular") {
                myCurrentEdge->evidence.oneway.emplace_back(
                    std::string("yes"), "junction=" + value, NIOSMConfidence::MEDIUM_HIGH);
            }
            if (value == "roundabout") {
                myCurrentEdge->myAmInRoundabout = true;
            }
        } else if (key == "oneway") {
            myCurrentEdge->myIsOneWay = value;
            myCurrentEdge->evidence.oneway.emplace_back(value, "oneway", NIOSMConfidence::HIGH);
        } else if (key == "name") {
            myCurrentEdge->streetName = value;
        } else if (key == "ref") {
            myCurrentEdge->ref = value;
            myCurrentEdge->setParameter("ref", value);
        } else if (key == "layer") {
            try {
                myCurrentEdge->myLayer = StringUtils::toInt(value);
            } catch (...) {
                WRITE_WARNINGF(TL("Value of key '%' is not numeric ('%') in edge '%'."), key, value, myCurrentEdge->id);
            }
        } else if (key == "tracks") {
            try {
                if (StringUtils::toInt(value) == 1) {
                    myCurrentEdge->myIsOneWay = "true";
                    myCurrentEdge->evidence.oneway.emplace_back(
                        std::string("yes"), "tracks=1", NIOSMConfidence::MEDIUM_HIGH);
                } else {
                    WRITE_WARNINGF(TL("Ignoring track count % for edge '%'."), value, myCurrentEdge->id);
                }
            } catch (...) {
                WRITE_WARNINGF(TL("Value of key '%' is not numeric ('%') in edge '%'."), key, value, myCurrentEdge->id);
            }
        } else if (key == "railway:preferred_direction") {
            if (value == "both") {
                myCurrentEdge->myRailDirection = WAY_BOTH | WAY_PREFER_FORWARD | WAY_PREFER_BACKWARD;
            } else if (value == "backward") {
                myCurrentEdge->myRailDirection = (myCurrentEdge->myRailDirection | WAY_BACKWARD | WAY_PREFER_BACKWARD) & ~WAY_UNKNOWN;
            } else if (value == "forward") {
                myCurrentEdge->myRailDirection = (myCurrentEdge->myRailDirection | WAY_FORWARD | WAY_PREFER_FORWARD) & ~WAY_UNKNOWN;
            }
        } else if (key == "railway:bidirectional") {
            if (value == "regular") {
                myCurrentEdge->myRailDirection = (myCurrentEdge->myRailDirection | WAY_BOTH) & ~WAY_UNKNOWN;
            }
        } else if (key == "electrified" || key == "segregated") {
            if (value != "no") {
                myCurrentEdge->myExtraTags[key] = value;
            }
        } else if (key == "railway:track_ref") {
            myCurrentEdge->setParameter(key, value);
        } else if (key == "public_transport" && value == "platform") {
            myCurrentEdge->myExtraTags["platform"] = "yes";
        } else if ((key == "parking:both" || key == "parking:lane:both") && !StringUtils::startsWith(value, "no")) {
            myCurrentEdge->myParkingType |= PARKING_BOTH;
        } else if ((key == "parking:left" || key == "parking:lane:left") && !StringUtils::startsWith(value, "no")) {
            myCurrentEdge->myParkingType |= PARKING_LEFT;
        } else if ((key == "parking:right" || key == "parking:lane:right") && !StringUtils::startsWith(value, "no")) {
            myCurrentEdge->myParkingType |= PARKING_RIGHT;
        } else if (key == "change" || key == "change:lanes") {
            myCurrentEdge->myChangeForward = myCurrentEdge->myChangeBackward = interpretChangeType(value);
        } else if (key == "change:forward" || key == "change:lanes:forward") {
            myCurrentEdge->myChangeForward = interpretChangeType(value);
        } else if (key == "change:backward" || key == "change:lanes:backward") {
            myCurrentEdge->myChangeBackward = interpretChangeType(value);
        } else if (key == "vehicle:lanes" || key == "vehicle:lanes:forward") {
            interpretLaneUse(value, SVC_PASSENGER, true);
            interpretLaneUse(value, SVC_PRIVATE, true);
            myCurrentEdge->evidence.lanesForward.emplace_back(
                (int)StringTokenizer(value, "|").getVector().size(),
                key + " pipe count", NIOSMConfidence::MEDIUM_HIGH);
        } else if (key == "vehicle:lanes:backward") {
            interpretLaneUse(value, SVC_PASSENGER, false);
            interpretLaneUse(value, SVC_PRIVATE, false);
            myCurrentEdge->evidence.lanesBackward.emplace_back(
                (int)StringTokenizer(value, "|").getVector().size(),
                key + " pipe count", NIOSMConfidence::MEDIUM_HIGH);
        } else if (key == "bus:lanes" || key == "bus:lanes:forward" || key == "lanes:bus" || key == "lanes:bus:forward") {
            try {
                const int cnt = StringUtils::toInt(value);
                myCurrentEdge->myBusLanesForwardCount = cnt;
                myCurrentEdge->myBusLanesForwardClasses |= SVC_BUS;
                myCurrentEdge->evidence.lanesForward.emplace_back(cnt, key + " count", NIOSMConfidence::HIGH);
            } catch (...) {
                interpretLaneUse(value, SVC_BUS, true);
                myCurrentEdge->evidence.lanesForward.emplace_back(
                    (int)StringTokenizer(value, "|").getVector().size(),
                    key + " pipe count", NIOSMConfidence::MEDIUM_HIGH);
            }
        } else if (key == "bus:lanes:backward" || key == "lanes:bus:backward") {
            try {
                const int cnt = StringUtils::toInt(value);
                myCurrentEdge->myBusLanesBackwardCount = cnt;
                myCurrentEdge->myBusLanesBackwardClasses |= SVC_BUS;
                myCurrentEdge->evidence.lanesBackward.emplace_back(cnt, key + " count", NIOSMConfidence::HIGH);
            } catch (...) {
                interpretLaneUse(value, SVC_BUS, false);
                myCurrentEdge->evidence.lanesBackward.emplace_back(
                    (int)StringTokenizer(value, "|").getVector().size(),
                    key + " pipe count", NIOSMConfidence::MEDIUM_HIGH);
            }
        } else if (key == "psv:lanes" || key == "psv:lanes:forward" || key == "lanes:psv" || key == "lanes:psv:forward") {
            try {
                const int cnt = StringUtils::toInt(value);
                myCurrentEdge->myBusLanesForwardCount = cnt;
                myCurrentEdge->myBusLanesForwardClasses |= (SVC_BUS | SVC_TAXI);
                myCurrentEdge->evidence.lanesForward.emplace_back(cnt, key + " count", NIOSMConfidence::HIGH);
            } catch (...) {
                interpretLaneUse(value, SVC_BUS, true);
                interpretLaneUse(value, SVC_TAXI, true);
                myCurrentEdge->evidence.lanesForward.emplace_back(
                    (int)StringTokenizer(value, "|").getVector().size(),
                    key + " pipe count", NIOSMConfidence::MEDIUM_HIGH);
            }
        } else if (key == "psv:lanes:backward" || key == "lanes:psv:backward") {
            try {
                const int cnt = StringUtils::toInt(value);
                myCurrentEdge->myBusLanesBackwardCount = cnt;
                myCurrentEdge->myBusLanesBackwardClasses |= (SVC_BUS | SVC_TAXI);
                myCurrentEdge->evidence.lanesBackward.emplace_back(cnt, key + " count", NIOSMConfidence::HIGH);
            } catch (...) {
                interpretLaneUse(value, SVC_BUS, false);
                interpretLaneUse(value, SVC_TAXI, false);
                myCurrentEdge->evidence.lanesBackward.emplace_back(
                    (int)StringTokenizer(value, "|").getVector().size(),
                    key + " pipe count", NIOSMConfidence::MEDIUM_HIGH);
            }
        } else if (key == "hgv:lanes" || key == "hgv:lanes:forward" || key == "lanes:hgv" || key == "lanes:hgv:forward") {
            interpretLaneUse(value, SVC_TRUCK, true);
            interpretLaneUse(value, SVC_TRAILER, true);
            myCurrentEdge->evidence.lanesForward.emplace_back(
                (int)StringTokenizer(value, "|").getVector().size(),
                key + " pipe count", NIOSMConfidence::MEDIUM_HIGH);
        } else if (key == "hgv:lanes:backward" || key == "lanes:hgv:backward") {
            interpretLaneUse(value, SVC_TRUCK, false);
            interpretLaneUse(value, SVC_TRAILER, false);
            myCurrentEdge->evidence.lanesBackward.emplace_back(
                (int)StringTokenizer(value, "|").getVector().size(),
                key + " pipe count", NIOSMConfidence::MEDIUM_HIGH);
        } else if (key == "taxi:lanes" || key == "taxi:lanes:forward" || key == "lanes:taxi" || key == "lanes:taxi:forward") {
            interpretLaneUse(value, SVC_TAXI, true);
            myCurrentEdge->evidence.lanesForward.emplace_back(
                (int)StringTokenizer(value, "|").getVector().size(),
                key + " pipe count", NIOSMConfidence::MEDIUM_HIGH);
        } else if (key == "taxi:lanes:backward" || key == "lanes:taxi:backward") {
            interpretLaneUse(value, SVC_TAXI, false);
            myCurrentEdge->evidence.lanesBackward.emplace_back(
                (int)StringTokenizer(value, "|").getVector().size(),
                key + " pipe count", NIOSMConfidence::MEDIUM_HIGH);
        } else if (key == "motorcycle:lanes" || key == "motorcycle:lanes:forward") {
            interpretLaneUse(value, SVC_MOTORCYCLE, true);
            myCurrentEdge->evidence.lanesForward.emplace_back(
                (int)StringTokenizer(value, "|").getVector().size(),
                key + " pipe count", NIOSMConfidence::MEDIUM_HIGH);
        } else if (key == "motorcycle:lanes:backward") {
            interpretLaneUse(value, SVC_MOTORCYCLE, false);
            myCurrentEdge->evidence.lanesBackward.emplace_back(
                (int)StringTokenizer(value, "|").getVector().size(),
                key + " pipe count", NIOSMConfidence::MEDIUM_HIGH);
        } else if (key == "moped:lanes" || key == "moped:lanes:forward") {
            interpretLaneUse(value, SVC_MOPED, true);
            myCurrentEdge->evidence.lanesForward.emplace_back(
                (int)StringTokenizer(value, "|").getVector().size(),
                key + " pipe count", NIOSMConfidence::MEDIUM_HIGH);
        } else if (key == "moped:lanes:backward") {
            interpretLaneUse(value, SVC_MOPED, false);
            myCurrentEdge->evidence.lanesBackward.emplace_back(
                (int)StringTokenizer(value, "|").getVector().size(),
                key + " pipe count", NIOSMConfidence::MEDIUM_HIGH);
        } else if (key == "motor_vehicle:lanes" || key == "motorcar:lanes"
                   || key == "motor_vehicle:lanes:forward" || key == "motorcar:lanes:forward") {
            // Apply to the motorised road classes as a group (matches the
            // semantics of the per-edge motor_vehicle=/motorcar= tags).
            for (const SUMOVehicleClass svc : {SVC_PASSENGER, SVC_HOV, SVC_TAXI, SVC_BUS,
                                               SVC_COACH, SVC_DELIVERY, SVC_TRUCK, SVC_TRAILER,
                                               SVC_MOTORCYCLE, SVC_MOPED, SVC_E_VEHICLE}) {
                interpretLaneUse(value, svc, true);
            }
            myCurrentEdge->evidence.lanesForward.emplace_back(
                (int)StringTokenizer(value, "|").getVector().size(),
                key + " pipe count", NIOSMConfidence::MEDIUM_HIGH);
        } else if (key == "motor_vehicle:lanes:backward" || key == "motorcar:lanes:backward") {
            for (const SUMOVehicleClass svc : {SVC_PASSENGER, SVC_HOV, SVC_TAXI, SVC_BUS,
                                               SVC_COACH, SVC_DELIVERY, SVC_TRUCK, SVC_TRAILER,
                                               SVC_MOTORCYCLE, SVC_MOPED, SVC_E_VEHICLE}) {
                interpretLaneUse(value, svc, false);
            }
            myCurrentEdge->evidence.lanesBackward.emplace_back(
                (int)StringTokenizer(value, "|").getVector().size(),
                key + " pipe count", NIOSMConfidence::MEDIUM_HIGH);
        } else if (key == "bicycle:lanes" || key == "bicycle:lanes:forward") {
            interpretLaneUse(value, SVC_BICYCLE, true);
        } else if (key == "bicycle:lanes:backward") {
            interpretLaneUse(value, SVC_BICYCLE, false);
        } else if (StringUtils::startsWith(key, "turn:") && key.find(":lanes") != std::string::npos) {
            int shift = 0;
            // use the first 8 bit to encode permitted directions for all classes
            // and the successive 8 bit blocks for selected classes
            if (StringUtils::startsWith(key, "turn:bus") || StringUtils::startsWith(key, "turn:psv")
                    || key.find(":bus:") != std::string::npos || key.find(":psv:") != std::string::npos
                    || StringUtils::endsWith(key, ":bus") || StringUtils::endsWith(key, ":psv")) {
                shift = NBEdge::TURN_SIGN_SHIFT_BUS;
            } else if (StringUtils::startsWith(key, "turn:taxi") || key.find(":taxi:") != std::string::npos
                       || StringUtils::endsWith(key, ":taxi")) {
                shift = NBEdge::TURN_SIGN_SHIFT_TAXI;
            } else if (StringUtils::startsWith(key, "turn:bicycle") || key.find(":bicycle:") != std::string::npos
                       || StringUtils::endsWith(key, ":bicycle")) {
                shift = NBEdge::TURN_SIGN_SHIFT_BICYCLE;
            }
            const std::vector<std::string> values = StringTokenizer(value, "|").getVector();
            std::vector<int> turnCodes;
            for (std::string codeList : values) {
                const std::vector<std::string> codes = StringTokenizer(codeList, ";").getVector();
                int turnCode = 0;
                if (codes.size() == 0) {
                    turnCode = (int)LinkDirection::STRAIGHT;
                }
                for (std::string code : codes) {
                    // Case-normalize before matching so YES/Yes/yes etc.
                    // all dispatch consistently.
                    std::transform(code.begin(), code.end(), code.begin(),
                                   [](unsigned char c) { return (char)std::tolower(c); });
                    int dir = niOSMTurnCodeToLinkDirection(code);
                    if (dir < 0) {
                        // Fuzzy auto-repair under --osm.repair=infer or
                        // aggressive. Levenshtein-distance-2 match against
                        // the canonical set, never replacing if multiple
                        // canonical candidates tie.
                        const std::string repair = OptionsCont::getOptions().getString("osm.repair");
                        if (repair == "infer" || repair == "aggressive") {
                            const std::string repaired = niOSMFuzzyMatchTurnCode(code);
                            if (!repaired.empty()) {
                                dir = niOSMTurnCodeToLinkDirection(repaired);
                                WRITE_WARNINGF(TL("Edge '%' turn:lanes code '%' auto-repaired to '%' (fuzzy match in tag %=%)."),
                                               toString(myCurrentEdge->id), code, repaired, key, value);
                            }
                        }
                        if (dir < 0) {
                            // Missing-pipe heuristic: if the unknown
                            // token splits cleanly into two canonical
                            // codes, surface that as a likely missing |.
                            std::string missingPipeFirst, missingPipeSecond;
                            for (size_t splitPos = 2; splitPos + 2 <= code.size(); ++splitPos) {
                                const std::string left = code.substr(0, splitPos);
                                const std::string right = code.substr(splitPos);
                                if (niOSMTurnCodeToLinkDirection(left) >= 0
                                        && niOSMTurnCodeToLinkDirection(right) >= 0) {
                                    missingPipeFirst = left;
                                    missingPipeSecond = right;
                                    break;
                                }
                            }
                            if (!missingPipeFirst.empty()) {
                                WRITE_WARNINGF(TL("Edge '%' turn:lanes code '%' looks like '%' and '%' concatenated without a '|' separator (in tag %=%); ignoring this entry."),
                                               toString(myCurrentEdge->id), code, missingPipeFirst, missingPipeSecond, key, value);
                            } else {
                                // Surface unknown values that we didn't
                                // repair (or repair was disabled).
                                WRITE_WARNINGF(TL("Edge '%' has unknown turn:lanes code '%' in tag %=%; ignoring this entry."),
                                               toString(myCurrentEdge->id), code, key, value);
                            }
                            continue;
                        }
                    }
                    turnCode |= dir << shift;
                }
                turnCodes.push_back(turnCode);
            }
            // An unsuffixed turn:lanes describes the way's traversal
            // direction. For a normal way that's the forward edge; for a
            // oneway=-1 / oneway=reverse way the actual driving direction
            // is the backward edge, so route it there. (Best-effort: if
            // the oneway tag is parsed *after* the turn:lanes tag in the
            // OSM XML stream, we'll have already routed to forward and
            // will miss the swap.)
            const bool reverseOneway = (myCurrentEdge->myIsOneWay == "-1"
                                        || myCurrentEdge->myIsOneWay == "reverse");
            const bool unsuffixed = StringUtils::endsWith(key, "lanes")
                                    && !StringUtils::endsWith(key, "lanes:forward")
                                    && !StringUtils::endsWith(key, "lanes:backward")
                                    && !StringUtils::endsWith(key, "lanes:both_ways");
            if (StringUtils::endsWith(key, "lanes:forward")
                    || (unsuffixed && !reverseOneway)) {
                mergeTurnSigns(myCurrentEdge->myTurnSignsForward, turnCodes);
            } else if (StringUtils::endsWith(key, "lanes:backward")
                       || (unsuffixed && reverseOneway)) {
                mergeTurnSigns(myCurrentEdge->myTurnSignsBackward, turnCodes);
            } else if (StringUtils::endsWith(key, "lanes:both_ways")) {
                mergeTurnSigns(myCurrentEdge->myTurnSignsForward, turnCodes);
                mergeTurnSigns(myCurrentEdge->myTurnSignsBackward, turnCodes);
            }
            // Pipe count witnesses lane count in the matching direction.
            // Only the unqualified turn:lanes variants count: class-qualified
            // forms (turn:bus:lanes, turn:taxi:lanes, turn:bicycle:lanes)
            // describe per-class sign assignments and don't witness total
            // lane count.
            if (shift == 0) {
                const int pipeCount = (int)values.size();
                // Same direction-routing rule as for the turn-sign vectors above.
                if (StringUtils::endsWith(key, "lanes:forward")
                        || (unsuffixed && !reverseOneway)) {
                    myCurrentEdge->evidence.lanesForward.emplace_back(
                        pipeCount, key + " pipe count", NIOSMConfidence::MEDIUM_HIGH);
                } else if (StringUtils::endsWith(key, "lanes:backward")
                           || (unsuffixed && reverseOneway)) {
                    myCurrentEdge->evidence.lanesBackward.emplace_back(
                        pipeCount, key + " pipe count", NIOSMConfidence::MEDIUM_HIGH);
                } else if (StringUtils::endsWith(key, "lanes:both_ways")) {
                    myCurrentEdge->evidence.lanesForward.emplace_back(
                        pipeCount, key + " pipe count", NIOSMConfidence::MEDIUM_HIGH);
                    myCurrentEdge->evidence.lanesBackward.emplace_back(
                        pipeCount, key + " pipe count", NIOSMConfidence::MEDIUM_HIGH);
                }
            }
        }
    }
}


void
NIImporter_OpenStreetMap::EdgesHandler::addType(const std::string& singleTypeID) {
    // special case: never build compound type for highspeed rail
    if (!myCurrentEdge->myHighWayType.empty() && singleTypeID != "railway.highspeed") {
        if (myCurrentEdge->myHighWayType == "railway.highspeed") {
            return;
        }
        // osm-ways may be used by more than one mode (eg railway.tram + highway.residential. this is relevant for multimodal traffic)
        // we create a new type for this kind of situation which must then be resolved in insertEdge()
        std::vector<std::string> types = StringTokenizer(myCurrentEdge->myHighWayType,
                                         compoundTypeSeparator).getVector();
        types.push_back(singleTypeID);
        myCurrentEdge->myHighWayType = joinToStringSorting(types, compoundTypeSeparator);
    } else {
        myCurrentEdge->myHighWayType = singleTypeID;
    }
}


double
NIImporter_OpenStreetMap::EdgesHandler::interpretSpeed(const std::string& key, std::string value) {
    if (mySpeedMap.find(value) != mySpeedMap.end()) {
        // sign / signals / nan indicate variable or sign-controlled limits
        // that the static network can't represent; the edge falls through
        // to the typemap class default. none / no are explicit "no posted
        // limit" tags (typical on German autobahn) and need no warning --
        // the substituted value is intentional.
        if (value == "sign" || value == "signals" || value == "nan") {
            WRITE_WARNINGF(TL("Edge '%' has %=% (variable / sign-controlled limit); preserved as <param>, edge speed falls back to highway-class default."),
                           toString(myCurrentEdge->id), key, value);
            myCurrentEdge->setParameter(key, value);
        }
        return mySpeedMap[value];
    } else {
        // handle symbolic names of the form DE:30 / DE:zone30 / DE:zone:30
        if (value.size() > 3 && value[2] == ':') {
            if (value.substr(3, 4) == "zone") {
                value = value.substr(7);
                if (!value.empty() && (value[0] == ':' || value[0] == '_')) {
                    value = value.substr(1);
                }
            } else {
                value = value.substr(3);
            }
        }
        try {
            return StringUtils::parseSpeed(value);
        } catch (...) {
            WRITE_WARNING("Value of key '" + key + "' is not numeric ('" + value + "') in edge '" +
                          toString(myCurrentEdge->id) + "'.");
            return MAXSPEED_UNGIVEN;
        }
    }
}


int
NIImporter_OpenStreetMap::EdgesHandler::interpretChangeType(const std::string& value) const {
    int result = 0;
    const std::vector<std::string> values = StringTokenizer(value, "|").getVector();
    for (const std::string& val : values) {
        if (val == "no") {
            result += CHANGE_NO;
        } else if (val == "not_left") {
            result += CHANGE_NO_LEFT;
        } else if (val == "not_right") {
            result += CHANGE_NO_RIGHT;
        }
        result = result << 2;
    }
    // last shift was superfluous
    result = result >> 2;

    if (values.size() > 1) {
        result |= (1 << 30); // mark multi-value input
    }
    //std::cout << " way=" << myCurrentEdge->id << " value=" << value << " result=" << std::bitset<32>(result) << "\n";
    return result;
}


bool
NIImporter_OpenStreetMap::EdgesHandler::interpretPlacement(const std::string& value, NIImporter_OpenStreetMap::PlacementType& placement, int& laneIndex) const {
    placement = NIImporter_OpenStreetMap::PlacementType::NONE;
    laneIndex = -1;
    const std::vector<std::string> tokens = StringTokenizer(value, ":").getVector();
    if (tokens.size() != 2) {
        return false;
    }
    const std::string where = StringUtils::prune(tokens[0]);
    if (where == "left_of") {
        placement = NIImporter_OpenStreetMap::PlacementType::LEFT_OF;
    } else if (where == "right_of") {
        placement = NIImporter_OpenStreetMap::PlacementType::RIGHT_OF;
    } else if (where == "middle_of") {
        placement = NIImporter_OpenStreetMap::PlacementType::MIDDLE_OF;
    } else {
        return false;
    }
    try {
        laneIndex = StringUtils::toInt(StringUtils::prune(tokens[1]));
    } catch (ProcessError&) {
        placement = NIImporter_OpenStreetMap::PlacementType::NONE;
        laneIndex = -1;
        return false;
    }
    if (laneIndex <= 0) {
        placement = NIImporter_OpenStreetMap::PlacementType::NONE;
        laneIndex = -1;
        return false;
    }
    return true;
}


void
NIImporter_OpenStreetMap::EdgesHandler::interpretLaneUse(const std::string& value, SUMOVehicleClass svc, const bool forward) const {
    const std::vector<std::string> values = StringTokenizer(value, "|").getVector();
    std::vector<bool>& designated = forward ? myCurrentEdge->myDesignatedLaneForward : myCurrentEdge->myDesignatedLaneBackward;
    std::vector<SVCPermissions>& allowed = forward ? myCurrentEdge->myAllowedLaneForward : myCurrentEdge->myAllowedLaneBackward;
    std::vector<SVCPermissions>& disallowed = forward ? myCurrentEdge->myDisallowedLaneForward : myCurrentEdge->myDisallowedLaneBackward;
    designated.resize(MAX2(designated.size(), values.size()), false);
    allowed.resize(MAX2(allowed.size(), values.size()), SVC_IGNORING);
    disallowed.resize(MAX2(disallowed.size(), values.size()), SVC_IGNORING);
    int i = 0;
    for (const std::string& rawVal : values) {
        std::string val = StringUtils::prune(rawVal);
        std::transform(val.begin(), val.end(), val.begin(),
                       [](unsigned char c) { return (char)std::tolower(c); });
        if (val.empty() || val == "none" || val == "default") {
            // Unspecified / default lane slot, keep edge default
        } else if (val == "yes" || val == "permissive") {
            allowed[i] |= svc;
        } else if (val == "lane" || val == "designated") {
            allowed[i] |= svc;
            designated[i] = true;
        } else if (val == "no") {
            disallowed[i] |= svc;
        } else {
            WRITE_WARNINGF(TL("Unknown lane use specifier '%' ignored for way '%'"), val, myCurrentEdge->id);
        }
        i++;
    }
}


void
NIImporter_OpenStreetMap::EdgesHandler::myEndElement(int element) {
    if (element == SUMO_TAG_WAY && myCurrentEdge != nullptr) {
        // Combined lifecycle and date-window inclusion policy.
        const std::string& simulatedDate = OptionsCont::getOptions().getString("osm.date");
        const DateStatus dateStatus = checkDateWindow(myCurrentEdge, simulatedDate);

        // Date-window check applies even to operational ways: a way with
        // end_date in the past should not be in the simulated network.
        if (dateStatus == DateStatus::OutOfRange && myCurrentEdge->myLifecycleStatus.empty()) {
            WRITE_MESSAGEF(TL("Discarding operational way '%' outside the simulated date window (start_date='%', end_date='%', --osm.date='%')."),
                           toString(myCurrentEdge->id),
                           myCurrentEdge->myStartDate,
                           myCurrentEdge->myEndDate,
                           simulatedDate);
            delete myCurrentEdge;
            myCurrentEdge = nullptr;
            return;
        }

        // Lifecycle policy. Default: drop non-operational ways with a
        // warning. Override: if --osm.date places the simulated date
        // inside the way's [start_date, end_date] window, include it as
        // operational.
        if (!myCurrentEdge->myLifecycleStatus.empty()) {
            // razed/demolished/removed ways describe roads that physically
            // no longer exist. They are always discarded regardless of
            // --osm.lifecycle and --osm.date settings -- there is no
            // geometry to include.
            const std::string& status = myCurrentEdge->myLifecycleStatus;
            if (status == "razed" || status == "demolished" || status == "removed") {
                if (OptionsCont::getOptions().getString("osm.lifecycle") == "warn") {
                    WRITE_WARNINGF(TL("Discarding physically-gone way '%' (lifecycle status: %)."),
                                   toString(myCurrentEdge->id), status);
                }
                delete myCurrentEdge;
                myCurrentEdge = nullptr;
                return;
            }
            const bool dateOverridesLifecycle = (dateStatus == DateStatus::InRange);
            if (!dateOverridesLifecycle) {
                const std::string lifecyclePolicy = OptionsCont::getOptions().getString("osm.lifecycle");
                if (lifecyclePolicy != "include") {
                    if (lifecyclePolicy == "warn") {
                        WRITE_WARNINGF(TL("Discarding non-operational way '%' (lifecycle status: %)."),
                                       toString(myCurrentEdge->id),
                                       myCurrentEdge->myLifecycleStatus);
                    }
                    delete myCurrentEdge;
                    myCurrentEdge = nullptr;
                    return;
                }
            } else {
                WRITE_MESSAGEF(TL("Including non-operational way '%' (lifecycle: %) because --osm.date=% falls within [start_date='%', end_date='%']."),
                               toString(myCurrentEdge->id),
                               myCurrentEdge->myLifecycleStatus,
                               simulatedDate,
                               myCurrentEdge->myStartDate,
                               myCurrentEdge->myEndDate);
            }
            // Record the lifecycle status as a parameter so downstream tools
            // can still see it whether we're including via 'include' or
            // via date override.
            myCurrentEdge->setParameter("osm.lifecycle", myCurrentEdge->myLifecycleStatus);
        }
        if (myCurrentEdge->myCurrentIsRoad) {
            // OSM-tag-evidence reconciliation, gated by --osm.repair so
            // default behaviour is unchanged. 'infer' and 'aggressive'
            // enable lane-count repair; 'warn' is observer-only.
            const std::string repair = OptionsCont::getOptions().getString("osm.repair");
            if (repair == "infer" || repair == "aggressive") {
                repairLaneCountFromWitnesses(myCurrentEdge);
            }
            if (repair != "off") {
                checkLaneCountBalance(myCurrentEdge);
                checkOnewayBackwardConflict(myCurrentEdge);
                checkWitnessAgreement(myCurrentEdge->evidence.lanesTotal,    "lanesTotal",    myCurrentEdge);
                checkWitnessAgreement(myCurrentEdge->evidence.lanesForward,  "lanesForward",  myCurrentEdge);
                checkWitnessAgreement(myCurrentEdge->evidence.lanesBackward, "lanesBackward", myCurrentEdge);
                checkWitnessAgreement(myCurrentEdge->evidence.speedForward,  "speedForward",  myCurrentEdge);
                checkWitnessAgreement(myCurrentEdge->evidence.speedBackward, "speedBackward", myCurrentEdge);
                checkLaneCountPlausibility(myCurrentEdge);
                checkSpeedPlausibility(myCurrentEdge);
                checkWidthPlausibility(myCurrentEdge);
                checkLifecycleDataQuality(myCurrentEdge,
                    OptionsCont::getOptions().getString("osm.date"));
            }
            const auto insertionIt = myEdgeMap.lower_bound(myCurrentEdge->id);
            if (insertionIt == myEdgeMap.end() || insertionIt->first != myCurrentEdge->id) {
                // assume we are loading multiple files, so we won't report duplicate edges
                myEdgeMap.emplace_hint(insertionIt, myCurrentEdge->id, myCurrentEdge);
            } else {
                delete myCurrentEdge;
            }
        } else if (myCurrentEdge->myExtraTags.count("platform") != 0) {
            const auto insertionIt = myPlatformShapesMap.lower_bound(myCurrentEdge->id);
            if (insertionIt == myPlatformShapesMap.end() || insertionIt->first != myCurrentEdge->id) {
                // assume we are loading multiple files, so we won't report duplicate platforms
                myPlatformShapesMap.emplace_hint(insertionIt, myCurrentEdge->id, myCurrentEdge);
            } else {
                delete myCurrentEdge;
            }
        } else {
            delete myCurrentEdge;
        }
        myCurrentEdge = nullptr;
    }
}


// ---------------------------------------------------------------------------
// definitions of NIImporter_OpenStreetMap::RelationHandler-methods
// ---------------------------------------------------------------------------
NIImporter_OpenStreetMap::RelationHandler::RelationHandler(
    const std::map<long long int, NIOSMNode*>& osmNodes,
    const std::map<long long int, Edge*>& osmEdges, NBPTStopCont* nbptStopCont,
    const std::map<long long int, Edge*>& platformShapes,
    NBPTLineCont* nbptLineCont,
    const OptionsCont& oc) :
    SUMOSAXHandler("osm - file"),
    myOSMNodes(osmNodes),
    myOSMEdges(osmEdges),
    myPlatformShapes(platformShapes),
    myNBPTStopCont(nbptStopCont),
    myNBPTLineCont(nbptLineCont),
    myOptionsCont(oc) {
    resetValues();
}


NIImporter_OpenStreetMap::RelationHandler::~RelationHandler() = default;


void
NIImporter_OpenStreetMap::RelationHandler::resetValues() {
    myCurrentRelation = INVALID_ID;
    myIsRestriction = false;
    myRestrictionException = SVC_IGNORING;
    myFromWay = INVALID_ID;
    myToWay = INVALID_ID;
    myViaNode = INVALID_ID;
    myExtraViaNodes.clear();
    myViaWays.clear();
    myStation = INVALID_ID;
    myRestrictionType = RestrictionType::UNKNOWN;
    myPlatforms.clear();
    myStops.clear();
    myPlatformStops.clear();
    myWays.clear();
    myIsStopArea = false;
    myIsRoute = false;
    myPTRouteType = "";
    myRouteColor.setValid(false);
}


void
NIImporter_OpenStreetMap::RelationHandler::myStartElement(int element, const SUMOSAXAttributes& attrs) {
    if (element == SUMO_TAG_RELATION) {
        bool ok = true;
        myCurrentRelation = attrs.get<long long int>(SUMO_ATTR_ID, nullptr, ok);
        const std::string& action = attrs.getOpt<std::string>(SUMO_ATTR_ACTION, nullptr, ok);
        if (action == "delete" || !ok) {
            myCurrentRelation = INVALID_ID;
        }
        myName = "";
        myRef = "";
        myInterval = -1;
        myNightService = "";
        return;
    }
    if (myCurrentRelation == INVALID_ID) {
        return;
    }
    if (element == SUMO_TAG_MEMBER) {
        bool ok = true;
        std::string role = attrs.hasAttribute("role") ? attrs.getStringSecure("role", "") : "";
        const long long int ref = attrs.get<long long int>(SUMO_ATTR_REF, nullptr, ok);
        if (role == "via") {
            // u-turns for divided ways may be given with 2 via-nodes or 1 via-way
            std::string memberType = attrs.get<std::string>(SUMO_ATTR_TYPE, nullptr, ok);
            if (memberType == "way" && checkEdgeRef(ref)) {
                myViaWays.push_back(ref);
            } else if (memberType == "node") {
                if (myOSMNodes.find(ref) != myOSMNodes.end()) {
                    if (myViaNode == INVALID_ID) {
                        myViaNode = ref;
                    } else {
                        // Multi-via-node case (e.g. divided-way u-turns).
                        // Stash extras; applyRestriction will use the
                        // last-listed via-node as the junction where the
                        // restriction is applied.
                        myExtraViaNodes.push_back(ref);
                    }
                } else {
                    WRITE_WARNINGF(TL("No node found for reference '%' in relation '%'."), toString(ref), toString(myCurrentRelation));
                }
            }
        } else if (role == "from" && checkEdgeRef(ref)) {
            myFromWay = ref;
        } else if (role == "to" && checkEdgeRef(ref)) {
            myToWay = ref;
        } else if (StringUtils::startsWith(role, "stop")) {
            // permit _entry_only and _exit_only variants
            myStops.push_back(ref);
        } else if (StringUtils::startsWith(role, "platform")) {
            // permit _entry_only and _exit_only variants
            std::string memberType = attrs.get<std::string>(SUMO_ATTR_TYPE, nullptr, ok);
            if (memberType == "way") {
                const std::map<long long int, NIImporter_OpenStreetMap::Edge*>::const_iterator& wayIt = myPlatformShapes.find(ref);
                if (wayIt != myPlatformShapes.end()) {
                    NIIPTPlatform platform;
                    platform.isWay = true;
                    platform.ref = ref;
                    myPlatforms.push_back(platform);
                }
            } else if (memberType == "node") {
                // myIsStopArea may not be set yet
                myStops.push_back(ref);
                myPlatformStops.insert(ref);
                NIIPTPlatform platform;
                platform.isWay = false;
                platform.ref = ref;
                myPlatforms.push_back(platform);
            }

        } else if (role == "station") {
            myStation = ref;
        } else if (role.empty()) {
            std::string memberType = attrs.get<std::string>(SUMO_ATTR_TYPE, nullptr, ok);
            if (memberType == "way") {
                myWays.push_back(ref);
            } else if (memberType == "node") {
                auto it = myOSMNodes.find(ref);
                if (it != myOSMNodes.end() && it->second->hasParameter("railway:ref")) {
                    myStation = ref;
                } else {
                    myStops.push_back(ref);
                }
            }
        }
        return;
    }
    // parse values
    if (element == SUMO_TAG_TAG) {
        bool ok = true;
        std::string key = attrs.get<std::string>(SUMO_ATTR_K, toString(myCurrentRelation).c_str(), ok, false);
        // we check whether the key is relevant (and we really need to transcode the value) to avoid hitting #1636
        if (key == "type" || key == "restriction") {
            std::string value = attrs.get<std::string>(SUMO_ATTR_V, toString(myCurrentRelation).c_str(), ok, false);
            if (key == "type" && value == "restriction") {
                myIsRestriction = true;
                return;
            }
            if (key == "type" && value == "route") {
                myIsRoute = true;
                return;
            }
            if (key == "restriction") {
                // @note: the 'right/left/straight' part is ignored since the information is
                // redundantly encoded in the 'from', 'to' and 'via' members
                if (value.substr(0, 5) == "only_") {
                    myRestrictionType = RestrictionType::ONLY;
                } else if (value.substr(0, 3) == "no_") {
                    myRestrictionType = RestrictionType::NO;
                } else {
                    WRITE_WARNINGF(TL("Found unknown restriction type '%' in relation '%'"), value, toString(myCurrentRelation));
                }
                return;
            }
        } else if (key == "except") {
            std::string value = attrs.get<std::string>(SUMO_ATTR_V, toString(myCurrentRelation).c_str(), ok, false);
            for (const std::string& v : StringTokenizer(value, ";").getVector()) {
                if (v == "psv") {
                    myRestrictionException |= SVC_BUS;
                } else if (v == "bicycle") {
                    myRestrictionException |= SVC_BICYCLE;
                } else if (v == "hgv") {
                    myRestrictionException |= SVC_TRUCK | SVC_TRAILER;
                } else if (v == "motorcar") {
                    myRestrictionException |= SVC_PASSENGER | SVC_TAXI;
                } else if (v == "emergency") {
                    myRestrictionException |= SVC_EMERGENCY;
                }
            }
        } else if (key == "public_transport") {
            std::string value = attrs.get<std::string>(SUMO_ATTR_V, toString(myCurrentRelation).c_str(), ok, false);
            if (value == "stop_area") {
                myIsStopArea = true;
            }
        } else if (key == "route") {
            std::string value = attrs.get<std::string>(SUMO_ATTR_V, toString(myCurrentRelation).c_str(), ok, false);
            if (value == "train" || value == "subway" || value == "light_rail" || value == "monorail" || value == "tram" || value == "bus"
                    || value == "trolleybus" || value == "aerialway" || value == "ferry" || value == "share_taxi" || value == "minibus") {
                myPTRouteType = value;
            }

        } else if (key == "name") {
            myName = attrs.get<std::string>(SUMO_ATTR_V, toString(myCurrentRelation).c_str(), ok, false);
        } else if (key == "colour") {
            std::string value = attrs.get<std::string>(SUMO_ATTR_V, toString(myCurrentRelation).c_str(), ok, false);
            try {
                myRouteColor = RGBColor::parseColor(value);
            } catch (...) {
                WRITE_WARNINGF(TL("Invalid color value '%' in relation %"), value, myCurrentRelation);
            }
        } else if (key == "ref") {
            myRef = attrs.get<std::string>(SUMO_ATTR_V, toString(myCurrentRelation).c_str(), ok, false);
        } else if (key == "interval" || key == "headway") {
            myInterval = attrs.get<int>(SUMO_ATTR_V, toString(myCurrentRelation).c_str(), ok, false);
        } else if (key == "by_night") {
            myNightService = attrs.get<std::string>(SUMO_ATTR_V, toString(myCurrentRelation).c_str(), ok, false);
        }
    }
}


bool
NIImporter_OpenStreetMap::RelationHandler::checkEdgeRef(long long int ref) const {
    if (myOSMEdges.find(ref) != myOSMEdges.end()) {
        return true;
    }
    WRITE_WARNINGF(TL("No way found for reference '%' in relation '%'"), toString(ref), toString(myCurrentRelation));
    return false;
}


void
NIImporter_OpenStreetMap::RelationHandler::myEndElement(int element) {
    if (element == SUMO_TAG_RELATION) {
        if (myIsRestriction) {
            assert(myCurrentRelation != INVALID_ID);
            bool ok = true;
            if (myRestrictionType == RestrictionType::UNKNOWN) {
                WRITE_WARNINGF(TL("Ignoring restriction relation '%' with unknown type."), toString(myCurrentRelation));
                ok = false;
            }
            if (myFromWay == INVALID_ID) {
                WRITE_WARNINGF(TL("Ignoring restriction relation '%' with unknown from-way."), toString(myCurrentRelation));
                ok = false;
            }
            if (myToWay == INVALID_ID) {
                WRITE_WARNINGF(TL("Ignoring restriction relation '%' with unknown to-way."), toString(myCurrentRelation));
                ok = false;
            }
            if (myViaNode == INVALID_ID && myViaWays.empty()) {
                WRITE_WARNINGF(TL("Ignoring restriction relation '%' with unknown via."), toString(myCurrentRelation));
                ok = false;
            }
            if (ok && !applyRestriction()) {
                WRITE_WARNINGF(TL("Ignoring restriction relation '%'."), toString(myCurrentRelation));
            }
        } else if (myIsStopArea) {
            for (long long ref : myStops) {
                myStopAreas[ref] = myCurrentRelation;
                if (myOSMNodes.find(ref) == myOSMNodes.end()) {
                    //WRITE_WARNING(
                    //    "Referenced node: '" + toString(ref) + "' in relation: '" + toString(myCurrentRelation)
                    //    + "' does not exist. Probably OSM file is incomplete.");
                    continue;
                }

                NIOSMNode* n = myOSMNodes.find(ref)->second;
                std::shared_ptr<NBPTStop> ptStop = myNBPTStopCont->get(toString(n->id));
                if (ptStop == nullptr) {
                    //WRITE_WARNING(
                    //    "Relation '" + toString(myCurrentRelation) + "' refers to a non existing pt stop at node: '"
                    //    + toString(n->id) + "'. Probably OSM file is incomplete.");
                    continue;
                }
                for (NIIPTPlatform& myPlatform : myPlatforms) {
                    if (myPlatform.isWay) {
                        assert(myPlatformShapes.find(myPlatform.ref) != myPlatformShapes.end()); //already tested earlier
                        Edge* edge = (*myPlatformShapes.find(myPlatform.ref)).second;
                        if (edge->myCurrentNodes.size() > 1 && edge->myCurrentNodes[0] == *(edge->myCurrentNodes.end() - 1)) {
                            WRITE_WARNINGF(TL("Platform '%' in relation: '%' is given as polygon, which currently is not supported."), myPlatform.ref, myCurrentRelation);
                            continue;

                        }
                        PositionVector p;
                        for (auto nodeRef : edge->myCurrentNodes) {
                            if (myOSMNodes.find(nodeRef) == myOSMNodes.end()) {
                                //WRITE_WARNING(
                                //    "Referenced node: '" + toString(ref) + "' in relation: '" + toString(myCurrentRelation)
                                //    + "' does not exist. Probably OSM file is incomplete.");
                                continue;
                            }
                            NIOSMNode* pNode = myOSMNodes.find(nodeRef)->second;
                            Position pNodePos(pNode->lon, pNode->lat, pNode->ele);
                            if (!NBNetBuilder::transformCoordinate(pNodePos)) {
                                WRITE_ERRORF("Unable to project coordinates for node '%'.", pNode->id);
                                continue;
                            }
                            p.push_back(pNodePos);
                        }
                        if (p.size() == 0) {
                            WRITE_WARNINGF(TL("Referenced platform: '%' in relation: '%' is corrupt. Probably OSM file is incomplete."),
                                           toString(myPlatform.ref), toString(myCurrentRelation));
                            continue;
                        }
                        NBPTPlatform platform(p[(int)p.size() / 2], p.length());
                        ptStop->addPlatformCand(platform);
                    } else {
                        if (myOSMNodes.find(myPlatform.ref) == myOSMNodes.end()) {
                            //WRITE_WARNING(
                            //    "Referenced node: '" + toString(ref) + "' in relation: '" + toString(myCurrentRelation)
                            //    + "' does not exist. Probably OSM file is incomplete.");
                            continue;
                        }
                        NIOSMNode* pNode = myOSMNodes.find(myPlatform.ref)->second;
                        Position platformPos(pNode->lon, pNode->lat, pNode->ele);
                        if (!NBNetBuilder::transformCoordinate(platformPos)) {
                            WRITE_ERRORF("Unable to project coordinates for node '%'.", pNode->id);
                        }
                        NBPTPlatform platform(platformPos, myOptionsCont.getFloat("osm.stop-output.length"));
                        ptStop->addPlatformCand(platform);

                    }
                }
                ptStop->setIsMultipleStopPositions(myStops.size() > 1, myCurrentRelation);
                if (myStation != INVALID_ID) {
                    const auto& nodeIt = myOSMNodes.find(myStation);
                    if (nodeIt != myOSMNodes.end()) {
                        NIOSMNode* station = nodeIt->second;
                        if (station != nullptr) {
                            if (station->hasParameter("railway:ref")) {
                                ptStop->setParameter("stationRef", station->getParameter("railway:ref"));
                            }
                        }
                    }
                }
            }
        } else if (myPTRouteType != "" && myIsRoute) {
            NBPTLine* ptLine = new NBPTLine(toString(myCurrentRelation), myName, myPTRouteType, myRef, myInterval, myNightService,
                                            interpretTransportType(myPTRouteType), myRouteColor);
            int consecutiveGap = false;
            int missingBefore = 0;
            int missingAfter = 0;
            for (long long ref : myStops) {
                const auto& nodeIt = myOSMNodes.find(ref);
                if (nodeIt == myOSMNodes.end()) {
                    if (ptLine->getStops().empty()) {
                        missingBefore++;
                    } else {
                        missingAfter++;
                        consecutiveGap++;
                    }
                    continue;
                }
                // give some slack for single missing stops
                if (consecutiveGap > 1) {
                    WRITE_WARNINGF(TL("PT line '%' in relation % has a gap of % stops, only keeping first part."), myName, myCurrentRelation, consecutiveGap);
                    missingAfter = (int)myStops.size() - missingBefore - (int)ptLine->getStops().size();
                    break;
                }
                // reset gap
                consecutiveGap = 0;

                const NIOSMNode* const n = nodeIt->second;
                std::shared_ptr<NBPTStop> ptStop = myNBPTStopCont->get(toString(n->id));
                if (ptStop == nullptr) {
                    // loose stop, which must later be mapped onto a line way
                    Position ptPos(n->lon, n->lat, n->ele);
                    if (!NBNetBuilder::transformCoordinate(ptPos)) {
                        WRITE_ERRORF("Unable to project coordinates for node '%'.", n->id);
                    }
                    const SumoXMLTag stopElement = isRailway(n->permissions) ? SUMO_TAG_TRAIN_STOP : SUMO_TAG_BUS_STOP;
                    ptStop = std::make_shared<NBPTStop>(stopElement, toString(n->id), ptPos, "", "", n->ptStopLength, n->name, n->permissions);
                    myNBPTStopCont->insert(ptStop);
                    if (myStopAreas.count(n->id)) {
                        ptStop->setIsMultipleStopPositions(false, myStopAreas[n->id]);
                    }
                    if (myPlatformStops.count(n->id) > 0) {
                        ptStop->setIsPlatform();
                    }
                }
                ptLine->addPTStop(ptStop);
            }
            for (long long& myWay : myWays) {
                auto entr = myOSMEdges.find(myWay);
                if (entr != myOSMEdges.end()) {
                    Edge* edge = entr->second;
                    for (long long& myCurrentNode : edge->myCurrentNodes) {
                        ptLine->addWayNode(myWay, myCurrentNode);
                    }
                }
            }
            ptLine->setNumOfStops((int)myStops.size(), missingBefore, missingAfter);
            if (ptLine->getStops().empty()) {
                WRITE_WARNINGF(TL("PT line in relation % with no stops ignored. Probably OSM file is incomplete."), myCurrentRelation);
                delete ptLine;
                resetValues();
                return;
            }
            if (!myNBPTLineCont->insert(ptLine)) {
                WRITE_WARNINGF(TL("Ignoring duplicate PT line '%'."), myCurrentRelation);
                delete ptLine;
            }
        }
        // other relations might use similar subelements so reset in any case
        resetValues();
    }
}

bool
NIImporter_OpenStreetMap::RelationHandler::applyRestriction() const {
    // since OSM ways are bidirectional we need the via to figure out which direction was meant
    if (myViaNode != INVALID_ID) {
        // For multi-via-node restrictions (divided-way u-turns and friends),
        // apply at the LAST via-node listed -- that's the junction where
        // to-way originates. Intermediate via-nodes mark the geometric
        // path but aren't enforced.
        const long long anchorNode = myExtraViaNodes.empty()
            ? myViaNode : myExtraViaNodes.back();
        if (!myExtraViaNodes.empty()) {
            WRITE_WARNINGF(TL("Restriction relation '%' has % via-nodes; applying restriction only at the final via -> to junction (node '%'). Intermediate via-nodes are not enforced."),
                           toString(myCurrentRelation),
                           toString(1 + (int)myExtraViaNodes.size()),
                           toString(anchorNode));
        }
        auto viaIt = myOSMNodes.find(anchorNode);
        if (viaIt == myOSMNodes.end() || viaIt->second == nullptr || viaIt->second->node == nullptr) {
            WRITE_WARNINGF(TL("Via-node '%' was not instantiated"), toString(anchorNode));
            return false;
        }
        NBNode* viaNode = viaIt->second->node;
        // For multi-via-node, look up the from-edge at the FIRST via-node
        // (where from-way ends) and to-edge at the LAST (anchor) via-node.
        // For single-via, both lookups happen at the same node.
        NBNode* fromAnchorNode = viaNode;
        if (!myExtraViaNodes.empty()) {
            auto firstIt = myOSMNodes.find(myViaNode);
            if (firstIt != myOSMNodes.end() && firstIt->second->node != nullptr) {
                fromAnchorNode = firstIt->second->node;
            }
        }
        NBEdge* from = findEdgeRef(myFromWay, fromAnchorNode->getIncomingEdges());
        NBEdge* to = findEdgeRef(myToWay, viaNode->getOutgoingEdges());
        if (from == nullptr) {
            WRITE_WARNINGF(TL("from-edge '%' of restriction relation could not be determined"), toString(myFromWay));
            return false;
        }
        if (to == nullptr) {
            WRITE_WARNINGF(TL("to-edge '%' of restriction relation could not be determined"), toString(myToWay));
            return false;
        }
        if (myRestrictionType == RestrictionType::ONLY) {
            from->addEdge2EdgeConnection(to, true);
            // make sure that these connections remain disabled even if network
            // modifications (ramps.guess) reset existing connections
            for (NBEdge* cand : from->getToNode()->getOutgoingEdges()) {
                if (!from->isConnectedTo(cand)) {
                    if (myRestrictionException == SVC_IGNORING) {
                        from->removeFromConnections(cand, -1, -1, true);
                    } else {
                        from->addEdge2EdgeConnection(cand, true, myRestrictionException);
                    }
                }
            }
        } else {
            if (myRestrictionException == SVC_IGNORING) {
                from->removeFromConnections(to, -1, -1, true);
            } else {
                from->addEdge2EdgeConnection(to, true, myRestrictionException);
                for (NBEdge* cand : from->getToNode()->getOutgoingEdges()) {
                    if (!from->isConnectedTo(cand)) {
                        from->addEdge2EdgeConnection(cand, true);
                    }
                }
            }
        }
    } else if (!myViaWays.empty()) {
        // Via-way restriction. The OSM convention is that the turn
        // happens at the (last) via-way -> to-way junction; we disable
        // the via-edge's connection to the to-edge (for no_*) or
        // constrain it to be the only outgoing connection (for only_*).
        // For multi-via-way restrictions we apply only at the final
        // junction; the intermediate via-ways are not enforced at the
        // SUMO level (no multi-step memory in connections). Surface this
        // approximation as a warning so the user knows.
        if (myViaWays.size() > 1) {
            WRITE_WARNINGF(TL("Restriction relation '%' has % via-ways; applying restriction only at the final via -> to junction. Intermediate via-ways are not enforced."),
                           toString(myCurrentRelation), toString((int)myViaWays.size()));
        }
        const long long int viaWayId = myViaWays.back();
        auto viaIt = myOSMEdges.find(viaWayId);
        if (viaIt == myOSMEdges.end() || viaIt->second->myCurrentNodes.size() < 2) {
            WRITE_WARNINGF(TL("Via-way '%' not found or has too few nodes for restriction relation '%'."),
                           toString(viaWayId), toString(myCurrentRelation));
            return false;
        }
        const long long firstNode = viaIt->second->myCurrentNodes.front();
        const long long lastNode = viaIt->second->myCurrentNodes.back();
        auto firstIt = myOSMNodes.find(firstNode);
        auto lastIt = myOSMNodes.find(lastNode);
        if (firstIt == myOSMNodes.end() || lastIt == myOSMNodes.end()
                || firstIt->second->node == nullptr || lastIt->second->node == nullptr) {
            WRITE_WARNINGF(TL("Via-way '%' endpoint nodes not instantiated for restriction relation '%'."),
                           toString(viaWayId), toString(myCurrentRelation));
            return false;
        }
        NBNode* nodeFirst = firstIt->second->node;
        NBNode* nodeLast = lastIt->second->node;

        // For multi-via, the "from" lookup compares against the *first*
        // via-way (which connects to from), but the via-edge we operate on
        // is the *last* via-way (which connects to to). Single-via case
        // reduces to from-way and via-way being adjacent.
        const long long int firstViaId = myViaWays.front();
        NBEdge* from = findEdgeRef(myFromWay, nodeFirst->getIncomingEdges());
        NBEdge* viaEdge = findEdgeRef(viaWayId, nodeFirst->getOutgoingEdges());
        NBEdge* to = findEdgeRef(myToWay, nodeLast->getOutgoingEdges());
        if (from == nullptr || viaEdge == nullptr || to == nullptr) {
            // try reverse via-way orientation
            from = findEdgeRef(myFromWay, nodeLast->getIncomingEdges());
            viaEdge = findEdgeRef(viaWayId, nodeLast->getOutgoingEdges());
            to = findEdgeRef(myToWay, nodeFirst->getOutgoingEdges());
        }
        if (myViaWays.size() > 1) {
            // For multi-via, from-way isn't adjacent to the LAST via-way;
            // we don't validate the from match in the chained case, only
            // the via->to anchor.
            (void)firstViaId; // silence unused warning if not needed below
            if (viaEdge == nullptr || to == nullptr) {
                WRITE_WARNINGF(TL("Could not locate final via/to edges for chained restriction relation '%' (last via-way '%')."),
                               toString(myCurrentRelation), toString(viaWayId));
                return false;
            }
        } else if (from == nullptr || viaEdge == nullptr || to == nullptr) {
            WRITE_WARNINGF(TL("Could not locate from/via/to edges for restriction relation '%' with via-way '%'."),
                           toString(myCurrentRelation), toString(viaWayId));
            return false;
        }
        if (myRestrictionType == RestrictionType::ONLY) {
            viaEdge->addEdge2EdgeConnection(to, true);
            for (NBEdge* cand : viaEdge->getToNode()->getOutgoingEdges()) {
                if (cand != to && !viaEdge->isConnectedTo(cand)) {
                    if (myRestrictionException == SVC_IGNORING) {
                        viaEdge->removeFromConnections(cand, -1, -1, true);
                    } else {
                        viaEdge->addEdge2EdgeConnection(cand, true, myRestrictionException);
                    }
                }
            }
        } else {
            if (myRestrictionException == SVC_IGNORING) {
                viaEdge->removeFromConnections(to, -1, -1, true);
            } else {
                viaEdge->addEdge2EdgeConnection(to, true, myRestrictionException);
            }
        }
    } else {
        // Multi-via-way / via-node-list restrictions still unsupported.
        WRITE_WARNINGF(TL("direction of restriction relation could not be determined%"), "");
        return false;
    }
    return true;
}

NBEdge*
NIImporter_OpenStreetMap::RelationHandler::findEdgeRef(long long int wayRef,
        const std::vector<NBEdge*>& candidates) const {
    const std::string prefix = toString(wayRef);
    const std::string backPrefix = "-" + prefix;
    NBEdge* result = nullptr;
    int found = 0;
    for (auto candidate : candidates) {
        const std::string& cid = candidate->getID();
        const bool matchFwd = (cid == prefix) || (cid.size() > prefix.size() && cid.compare(0, prefix.size(), prefix) == 0 && cid[prefix.size()] == '#');
        const bool matchBwd = (cid == backPrefix) || (cid.size() > backPrefix.size() && cid.compare(0, backPrefix.size(), backPrefix) == 0 && cid[backPrefix.size()] == '#');
        if (matchFwd || matchBwd) {
            result = candidate;
            found++;
        }
    }
    if (found > 1) {
        WRITE_WARNINGF(TL("Ambiguous way reference '%' in restriction relation"), prefix);
        result = nullptr;
    }
    return result;
}


/****************************************************************************/
