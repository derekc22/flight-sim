from analysis.linear.frequency_domain import run as run_frequency_domain
from analysis.linear.time_domain import run as run_time_domain


RUNNERS = {
    "time_domain": run_time_domain,
    "frequency_domain": run_frequency_domain,
}
