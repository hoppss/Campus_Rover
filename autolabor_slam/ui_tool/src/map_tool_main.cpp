#include <QApplication>
#include <map_tool_ui.hpp>

int main(int argc, char** argv)
{
  QApplication app(argc, argv);
  MapToolUi s(argc, argv);
  return app.exec();
}
