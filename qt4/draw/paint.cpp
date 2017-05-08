#include <QtGui>
#include "draw.h"

MyWidget::MyWidget(QWidget* parent) : QWidget(parent)
{
	m_pPaintBtn = new QPushButton(tr("&Paint"));

}
