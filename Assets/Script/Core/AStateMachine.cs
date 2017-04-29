using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System;
/// <summary>
/// Atwood's state machine class
/// </summary>
public class AStateMachine<StateType,EventType> 
	where StateType : struct , IFormattable , IConvertible , IComparable
	where EventType : struct , IFormattable , IConvertible , IComparable {

	private StateType m_state;
	public bool enable = true;
	/// <summary>
	/// the interface for setting and getting the state of the state machine
	/// </summary>
	/// <value>The state.</value>
	public StateType State
	{
		get {
			return m_state;
		}
		set {
			if ( !value.Equals(m_state) && enable) {
				if ( exitState.ContainsKey(m_state) && exitState [m_state] != null )
					exitState [m_state]();

				if (stateChangeEvent != null) {
					stateChangeEvent (m_state, value);
				}
				m_state = value;

				if (timeEventDict.ContainsKey (m_state)) {
					innerTimer = timeEventDict [m_state].Value;
				} else {
					innerTimer = -999f;
				}

				if ( enterState.ContainsKey(m_state) && enterState [m_state] != null )
					enterState [m_state]();
			}
		}
	}

	/// <summary>
	/// The inner timer for the time based event.
	/// </summary>
	private float innerTimer = -999f;

	private Dictionary<StateType,Action> enterState = new Dictionary<StateType, Action>();
	private Dictionary<StateType,Action> updateState = new Dictionary<StateType, Action>();
	private Dictionary<StateType,Action> exitState = new Dictionary<StateType, Action>();

	public delegate void StateChangeHandler( StateType fromState, StateType toState);
	private event StateChangeHandler stateChangeEvent;

	private Dictionary<EventType,List<KeyValuePair<StateType,StateType>>> eventDict = new Dictionary<EventType, List<KeyValuePair<StateType, StateType>>> ();
	private Dictionary<StateType,KeyValuePair<StateType, float>> timeEventDict = new Dictionary<StateType, KeyValuePair<StateType, float>>();
	private Dictionary<EventType,StateType> everyStateDict = new Dictionary<EventType, StateType>();

	public AStateMachine( StateType initState )
	{
		m_state = initState;
	}

	/// <summary>
	/// Adds and enter function for the state
	/// </summary>
	/// <param name="type">Type.</param>
	/// <param name="func">Func.</param>
	public void AddEnter( StateType type , Action func )
	{
		if (enterState.ContainsKey (type))
			enterState [type] += func;
		else {
			enterState [type] = func;
		}
	}

	/// <summary>
	/// Adds the update function for the state
	/// </summary>
	/// <param name="type">Type.</param>
	/// <param name="func">Func.</param>
	public void AddUpdate( StateType type , Action func )
	{
		if (updateState.ContainsKey (type))
			updateState [type] += func;
		else {
			updateState [type] = func;
		}
	}

	/// <summary>
	/// Adds the exit function for the state.
	/// </summary>
	/// <param name="type">Type.</param>
	/// <param name="func">Func.</param>
	public void AddExit( StateType type , Action func )
	{
		if (exitState.ContainsKey (type))
			exitState [type] += func;
		else {
			exitState [type] = func;
		}
	}

	public void AddOnChange(StateChangeHandler func)
	{
		stateChangeEvent += func;
	}

	/// <summary>
	/// Update the state machine
	/// Call this function to active all the update delegates
	/// </summary>
	public void Update()
	{
		if (updateState.ContainsKey (State))
			updateState [State] ();

		float timerTo = innerTimer - Time.deltaTime;
		// timer change to 0
		if (innerTimer * timerTo < 0) {
			if (timeEventDict.ContainsKey (State)) {
				State = timeEventDict [State].Key;
			}
		} else {
			innerTimer = timerTo;
		}
	}

	/// <summary>
	/// Blinds the state change event to the from and to states
	/// remember to call ReactEvent when recieve an event 
	/// </summary>
	/// <param name="type">Type.</param>
	/// <param name="fromState">From state.</param>
	/// <param name="toState">To state.</param>
	public void BlindStateChangeEvent( EventType type , StateType fromState , StateType toState )
	{
		if (everyStateDict.ContainsKey (type)) {
			Debug.LogError ("An Every State Event exists :" + type );
		}
		var pair = new KeyValuePair<StateType,StateType> (fromState, toState);
		if (!eventDict.ContainsKey (type)) {
			eventDict [type] = new List<KeyValuePair<StateType, StateType>> ();
		}
		eventDict [type].Add (pair);
	}

	/// <summary>
	/// Blinds the time state change events
	/// </summary>
	/// <returns><c>true</c>, if time state change was blinded, <c>false</c> if there is already a time event for from state.</returns>
	/// <param name="time">Time.</param>
	/// <param name="fromState">From state.</param>
	/// <param name="toState">To state.</param>
	public bool BlindTimeStateChange( StateType fromState , StateType toState , float time)
	{
		if (timeEventDict.ContainsKey (fromState))
			return false;
		var pair = new KeyValuePair<StateType,float> (toState, time);
		timeEventDict [fromState] = pair;
		return true;
	}

	public void BlindFromEveryState( EventType type , StateType toState )
	{
		if (everyStateDict.ContainsKey (type)) {
			Debug.LogError ("An Every State Event exists :" + type );
		}

		if (eventDict.ContainsKey (type)) {
			Debug.LogError ("The Event has already registered" + type );

		}

		everyStateDict.Add (type, toState);
	}

	/// <summary>
	/// React to the trigger events
	/// </summary>
	/// <param name="type">Type.</param>
	public void OnEvent(EventType type)
	{
		// if the event is registered in the normal dict
		if (eventDict.ContainsKey (type)) {
			// get the state jumping list
			var list = eventDict [type];
			// check each pair
			foreach (var pair in list) {
				if (pair.Key.Equals( State)) {
					// jump to next state
					State = pair.Value;
					// one state move each time
					return;
				}
			}
		}

		// if the event is registered in every state dict
		if (everyStateDict.ContainsKey (type)) {
			State = everyStateDict [type];
		}
	}
}