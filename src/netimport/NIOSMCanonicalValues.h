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
/// @file    NIOSMCanonicalValues.h
/// @date    2026-05-12
///
// Centralised canonical OSM tag value sets and the helpers that consume
// them: exact dispatch, fuzzy auto-repair (Levenshtein), missing-pipe
// heuristic. Extracted so the parser, the warning sites, and the
// auto-repair code share a single source of truth.
//
// Currently covers turn:lanes codes (the most-extended canonical set);
// oneway / access / bus values are still scattered across the parser's
// if/else chains and could be migrated here as the same patterns
// (unknown-value warning, fuzzy match, case normalisation) are
// extended to them.
/****************************************************************************/
#pragma once
#include <config.h>

#include <algorithm>
#include <string>
#include <vector>
#include <utils/xml/SUMOXMLDefinitions.h>


/// Standard dynamic-programming Levenshtein edit distance. O(|a|*|b|).
inline int niOSMLevenshteinDistance(const std::string& a, const std::string& b) {
    const size_t m = a.size();
    const size_t n = b.size();
    if (m == 0) {
        return (int)n;
    }
    if (n == 0) {
        return (int)m;
    }
    std::vector<int> prev(n + 1), cur(n + 1);
    for (size_t j = 0; j <= n; ++j) {
        prev[j] = (int)j;
    }
    for (size_t i = 1; i <= m; ++i) {
        cur[0] = (int)i;
        for (size_t j = 1; j <= n; ++j) {
            const int cost = (a[i - 1] == b[j - 1]) ? 0 : 1;
            cur[j] = std::min({prev[j] + 1, cur[j - 1] + 1, prev[j - 1] + cost});
        }
        std::swap(prev, cur);
    }
    return prev[n];
}

/// Map a canonical turn:lanes code to its LinkDirection-bit value. Returns
/// -1 for unknown codes. Centralised so the exact-match dispatch and the
/// fuzzy-repaired retry share one source of truth.
inline int niOSMTurnCodeToLinkDirection(const std::string& code) {
    if (code.empty() || code == "none" || code == "through"
            || code == "straight" || code == "forward") {
        return (int)LinkDirection::STRAIGHT;
    }
    if (code == "left" || code == "sharp_left") {
        return (int)LinkDirection::LEFT;
    }
    if (code == "right" || code == "sharp_right") {
        return (int)LinkDirection::RIGHT;
    }
    if (code == "slight_left") {
        return (int)LinkDirection::PARTLEFT;
    }
    if (code == "slight_right") {
        return (int)LinkDirection::PARTRIGHT;
    }
    if (code == "reverse" || code == "u_turn") {
        return (int)LinkDirection::TURN;
    }
    if (code == "merge_to_left" || code == "merge_to_right") {
        return (int)LinkDirection::NODIR;
    }
    return -1;
}

/// Try to repair an unknown turn:lanes code by Levenshtein-matching it to
/// the canonical set. Returns the canonical code if exactly one canonical
/// is within distance 2 (no ties). Empty string otherwise.
inline std::string niOSMFuzzyMatchTurnCode(const std::string& code) {
    static const char* const kCanonicals[] = {
        "none", "through", "straight", "forward",
        "left", "sharp_left", "right", "sharp_right",
        "slight_left", "slight_right",
        "reverse", "u_turn",
        "merge_to_left", "merge_to_right"
    };
    int bestDistance = 3;
    std::string bestMatch;
    bool tied = false;
    for (const char* const c : kCanonicals) {
        const int d = niOSMLevenshteinDistance(code, c);
        if (d < bestDistance) {
            bestDistance = d;
            bestMatch = c;
            tied = false;
        } else if (d == bestDistance) {
            tied = true;
        }
    }
    if (bestDistance > 2 || tied) {
        return "";
    }
    return bestMatch;
}
