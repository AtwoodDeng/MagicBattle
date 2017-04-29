using UnityEngine;
using System.Collections;

public class MBehavior : MonoBehaviour {

	void Awake()
	{
		MAwake ();
	}

	void Start()
	{
		MStart ();
	}

	void Update()
	{
		MUpdate ();
	}

	void OnEnable()
	{
		MOnEnable ();
	}

	void OnDisable()
	{
		MOnDisable ();
	}

	virtual protected void MAwake() {

	}

	// Use this for initialization
	virtual protected void MStart () {
		
	}
	
	// Update is called once per frame
	virtual protected void MUpdate () {
	
	}

	virtual protected void MOnEnable() {
	}

	virtual protected void MOnDisable() {
	}

}
