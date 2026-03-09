"""Application entrypoint."""

import logging
import signal
import sys

from app.orchestration.mission_orchestrator import MissionOrchestrator
from app.web.dashboard import DashboardServer


logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
)
logger = logging.getLogger(__name__)


def main() -> None:
    orchestrator = MissionOrchestrator()
    dashboard = DashboardServer(orchestrator)

    def _shutdown(*_args):
        logger.info("Shutdown requested")
        orchestrator.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    orchestrator.start_background_services()
    dashboard.run()


if __name__ == "__main__":
    main()
