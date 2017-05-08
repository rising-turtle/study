#include <QtGui>
#include "find.h"
#include <iostream>

FindDialog::FindDialog(QWidget* parent):QDialog(parent)
{
	m_plabel = new QLabel(tr("Find &what:"));
	m_plineEdit = new QLineEdit;
	m_plabel->setBuddy(m_plineEdit);

	m_pcheckBox = new QCheckBox(tr("Match &case"));
	m_pbackcheckBox = new QCheckBox(tr("Search &backward"));

	m_pfindButton = new QPushButton(tr("&Find"));
	m_pcloseButton = new QPushButton(tr("Close"));
	m_pfindButton->setDefault(true);
	m_pfindButton->setEnabled(false);

	
	connect(m_plineEdit,SIGNAL(textChanged(const QString &)),
		this,SLOT(enableFindButton(const QString& )));
	connect(m_pfindButton,SIGNAL(clicked()),
		this,SLOT(findClicked()));
	connect(m_pcloseButton,SIGNAL(clicked()),
		this,SLOT(close()));

	QHBoxLayout *topLeftLayout = new QHBoxLayout;
	topLeftLayout->addWidget(m_plabel);
	topLeftLayout->addWidget(m_plineEdit);

	QVBoxLayout *leftLayout = new QVBoxLayout;
	leftLayout->addLayout(topLeftLayout);
	leftLayout->addWidget(m_pcheckBox);
	leftLayout->addWidget(m_pbackcheckBox);

	QVBoxLayout *rightLayout = new QVBoxLayout;
	rightLayout->addWidget(m_pfindButton);
	rightLayout->addWidget(m_pcloseButton);

	QHBoxLayout *mainLayout = new QHBoxLayout;
	mainLayout->addLayout(leftLayout);
	mainLayout->addLayout(rightLayout);
	setLayout(mainLayout);
	setWindowTitle(tr("Find"));
	setFixedHeight(sizeHint().height());
}

void FindDialog::findClicked()
{
	QString text = m_plineEdit->text();
	Qt::CaseSensitivity cs = m_pcheckBox->isChecked()?Qt::CaseSensitive:Qt::CaseInsensitive;

	if(m_pbackcheckBox->isChecked()){
		//emit findPrevious(text,cs);
	}else{
		//emit findNext(text,cs);
	}
}

void FindDialog::enableFindButton(const QString& text)
{
	m_pfindButton->setEnabled(!text.isEmpty());
}

