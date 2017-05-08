
template<typename T>
void CTemplate1<T>::type_Independent()
{
	cout<<"type_independent: "<<foo(_i1)<<endl;
}

template<typename T>
void CTemplate1<T>::type_dependent()
{
	cout<<"type_dependent: "<<foo(_v1)<<endl;
}