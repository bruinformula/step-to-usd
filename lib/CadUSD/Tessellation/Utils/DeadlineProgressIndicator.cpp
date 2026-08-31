
#include <chrono>
#include <Message_ProgressIndicator.hxx>

#include "CadUSD/Logger.h"
#include "CadUSD/Tessellation/DeadlineProgressIndicator.h"

DeadlineProgressIndicator::~DeadlineProgressIndicator() {
    if (_timedOut) {
        LOG_WARN(label + "DeadlineProgressIndicator destroyed after timeout");
    } else {
        // LOG_DEBUG(label + "DeadlineProgressIndicator destroyed (completed within deadline)");
    }
}

// OCCT polls this at internal checkpoints
bool DeadlineProgressIndicator::UserBreak() {
    if (Clock::now() >= _deadline) {
        if (!_timedOut) {  // Only log once on first timeout detection
            LOG_WARN(label + "operation timed out");
        }
        _timedOut = true;
        return true;
    }
    // LOG_DEBUG(label + "UserBreak polled, still within deadline");
    return false;
}

void DeadlineProgressIndicator::Show(const Message_ProgressScope& scope, const bool isForced) {
    // LOG_DEBUG(label + "Show called (forced=" + std::string(isForced ? "true" : "false") + ")");
}

bool DeadlineProgressIndicator::timedOut() const { return _timedOut; }
