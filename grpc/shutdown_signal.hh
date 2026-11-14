#pragma once

namespace msensor {

/**
 * @brief Installs SIGINT and SIGTERM handlers and blocks until one of them
 * arrives, then restores the previous handlers and returns.
 *
 * The handler only sets a flag (async-signal-safe); the caller's thread
 * notices it within a few tens of milliseconds. Meant for the main thread of
 * an application: start the server, call this, then stop the server and return
 * from `main` so the destructors run. Not reentrant: call from one thread at a
 * time.
 */
void waitForShutdownSignal();

} // namespace msensor
