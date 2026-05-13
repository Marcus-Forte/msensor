#include "msensor/interface/IFile.hh"
#include <gmock/gmock.h>

using namespace msensor;

/// GoogleMock-based mock for `IFile` used in unit tests.
class IFileMock : public IFile {
public:
  MOCK_METHOD(void, open, (const std::string &filename), (override));
  MOCK_METHOD(void, write, (const char *data, size_t size), (override));
  MOCK_METHOD(void, close, (), (override));
  MOCK_METHOD(std::ostream *, ostream, (), (override));
};