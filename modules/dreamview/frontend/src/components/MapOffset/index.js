import React from "react";
import { MAP_WS } from "store/websocket";

const IncrementStep = 0.000001;

export default class MapOffset extends React.PureComponent {
    constructor(){
        super();
        this.state = {
          lat: 0,
          lon: 1
        };
        this.handleChange = this.handleChange.bind(this);
        this.decLat = this.decLat.bind(this);
        this.incLat = this.incLat.bind(this);
        this.decLon = this.decLon.bind(this);
        this.incLon = this.incLon.bind(this);
    }

    handleChange (evt) {
        this.setState({ [evt.target.name]: parseFloat(evt.target.value) });
    }

    setValues = evt => {
        //this routine is done due to keep
        //state var as floats, but send as string (crazy)
        const state={
            lat: this.state.lat+'',
            lon: this.state.lon+''
        };
        MAP_WS.changeMapOffset(state);
    }

    decLat() {
        this.setState(lat => ({
            lat: this.state.lat - IncrementStep,
        }));
        this.setValues();
      }

    incLat() {
        this.setState(lat => ({
            lat: this.state.lat + IncrementStep,
        }));
        this.setValues();
    }

    decLon() {
        this.setState(lat => ({
            lon: this.state.lon - IncrementStep,
        }));
        this.setValues();
      }


    incLon() {
        this.setState(lat => ({
            lon: this.state.lon + IncrementStep,
        }));
        this.setValues();
    }

    render() {
        return (
            <div className="offset-bar" >
                <div className="offset-window">
                    <div className="text">
                        <div className="label">lat: </div>
                        <input type="text" name="lat" tabindex="1"
                    onChange={this.handleChange} value={this.state.lat} />
                        <button onClick={this.incLat}>+</button>
                        <button onClick={this.decLat}>-</button>
                    </div>
                    <div className="text">
                        <div className="label">lon: </div>
                        <input type="text" name="lon" tabindex="2"
                    onChange={this.handleChange} value={this.state.lon} />
                        <button onClick={this.incLon}>+</button>
                        <button onClick={this.decLon}>-</button>
                    </div>
                    <button onClick={this.setValues} tabindex="3">Set!</button>
                </div>
            </div>
        );
    }
}