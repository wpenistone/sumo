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
/// @file    NIOSMTagEvidence.h
/// @date    2026-05-12
///
// Per-edge OSM-tag evidence collected during import for cross-tag
// reconciliation. Witness lists are populated alongside the existing
// direct-write parsing in NIImporter_OpenStreetMap and consumed by the
// observer/repair functions that watch for inconsistencies and (under
// --osm.repair=infer) derive missing values from related tags.
/****************************************************************************/
#pragma once
#include <config.h>

#include <string>
#include <utility>
#include <vector>


/// @brief Confidence level for a single OSM-tag-derived witness. Higher
///        levels override lower ones in the default conflict policy.
enum class NIOSMConfidence : int {
    /// Highway-class typemap default (last-resort fallback).
    LOWEST = 0,
    /// Heuristic inference (e.g. capping speed by surface or traffic_calming).
    LOW = 1,
    /// Indirect inference from a related tag.
    MEDIUM = 2,
    /// Per-lane pipe counts, country-default speed lookups, derived via a
    /// conservation constraint from other High-confidence witnesses.
    MEDIUM_HIGH = 3,
    /// Direct, explicit OSM tag.
    HIGH = 4,
};


/// @brief One witness for some derived edge attribute. Carries the value
///        plus the OSM tag it came from and a confidence level.
template <typename T>
struct NIOSMEvidence {
    T value;
    std::string sourceTag;
    NIOSMConfidence confidence;

    NIOSMEvidence(const T& v, std::string tag, NIOSMConfidence c) :
        value(v), sourceTag(std::move(tag)), confidence(c) {}
};


/// @brief Per-edge collection of OSM-tag evidence. Witness lists grow as
///        the parse loop encounters relevant tags. Observer functions
///        compare witnesses against each other to surface inconsistencies;
///        repair functions consume them to derive missing values when
///        --osm.repair=infer is set.
struct NIOSMTagEvidence {
    /// Witnesses for total lane count on the edge.
    std::vector<NIOSMEvidence<int>> lanesTotal;
    /// Witnesses for forward-direction lane count.
    std::vector<NIOSMEvidence<int>> lanesForward;
    /// Witnesses for backward-direction lane count.
    std::vector<NIOSMEvidence<int>> lanesBackward;
    /// Witnesses for center two-way (both-ways) lane count.
    std::vector<NIOSMEvidence<int>> lanesBothWays;

    /// Witnesses for forward-direction speed (m/s as parsed by interpretSpeed).
    std::vector<NIOSMEvidence<double>> speedForward;
    /// Witnesses for backward-direction speed (m/s).
    std::vector<NIOSMEvidence<double>> speedBackward;

    /// Witnesses for total carriageway width (metres).
    std::vector<NIOSMEvidence<double>> width;

    /// Witnesses for the oneway state. Stored as the raw OSM string value
    /// (yes/no/true/false/-1/reverse/...) so the resolver can preserve
    /// direction semantics like -1 and reverse. Implicit witnesses
    /// (junction=roundabout, tracks=1, highway-class typemap) emit "yes".
    std::vector<NIOSMEvidence<std::string>> oneway;

    /// Per-mode oneway exceptions: oneway:bus=no, oneway:psv=no, etc.
    /// The value of each witness is the mode name (e.g. "bus", "psv"); the
    /// presence of any exception means the way has a contraflow lane (or
    /// shared-lane bidirectional travel) for the named mode despite an
    /// otherwise-oneway state.
    std::vector<NIOSMEvidence<std::string>> onewayExceptions;
};
