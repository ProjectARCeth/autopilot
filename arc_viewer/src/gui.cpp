#include <QApplication>
#include "ControlWindow.hpp"

int main(int argc, char** argv) {
    QApplication app(argc, argv);
    ControlWindow window(argc, argv);
    return app.exec();
}
