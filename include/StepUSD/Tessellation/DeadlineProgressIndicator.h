#pragma once 

#include <chrono>
#include <Message_ProgressIndicator.hxx>
 
class DeadlineProgressIndicator : public Message_ProgressIndicator {
public:
    using Clock = std::chrono::steady_clock;
    using Duration = std::chrono::milliseconds;

    explicit DeadlineProgressIndicator(Duration timeout, std::string partLabel) : 
        _deadline(Clock::now() + timeout),
        _timedOut(false),
        label(partLabel) 
    {
    }

    ~DeadlineProgressIndicator();

    // OCCT polls this at internal checkpoints
    bool UserBreak() override;

    void Show(const Message_ProgressScope& scope, const bool isForced) override;
    bool timedOut() const;

private:
    std::chrono::time_point<Clock> _deadline;
    bool _timedOut;
    std::string label;
};

template<typename Performer>
bool runWithDeadline(
    std::chrono::milliseconds timeout,
    const std::string& label,
    Performer& performer
) {
    auto progress = new DeadlineProgressIndicator(timeout, label);
    Message_ProgressRange range = progress->Start();

    performer.Perform(range);

    return progress->timedOut();
}