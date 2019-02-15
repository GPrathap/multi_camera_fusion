import React from "react";
import { MAP_WS } from "store/websocket";
import { inject, observer} from "mobx-react";
import STORE from "store";

const IncrementStep = 0.000001;

@inject("store") @observer
export default class MapOffset extends React.PureComponent {
    constructor(){
        super();
        this.handleChange = this.handleChange.bind(this);
        this.decLat = this.decLat.bind(this);
        this.incLat = this.incLat.bind(this);
        this.decLon = this.decLon.bind(this);
        this.incLon = this.incLon.bind(this);
    }

    getOffsetPureObject () {
        const obj = {
            lat: this.props.store.mapGeoOffsets.lat,
            lon: this.props.store.mapGeoOffsets.lon
        };
        return obj;
    }
    handleChange (evt) {
        // this.setState({ [evt.target.name]: parseFloat(evt.target.value) });
        const obj = this.getOffsetPureObject();
        obj[evt.target.name] = parseFloat(evt.target.value);
        STORE.updateMapOffsets(obj);
    }

    componentDidMount() {
        MAP_WS.requestMapOffset();
        // this.state = {
        //     lat: this.props.store.mapGeoOffsets.lat,
        //     lon: this.props.store.mapGeoOffsets.lon
        // };
    }

    setValues = evt => {
        //this routine is done due to keep
        //state var as floats, but send as string (crazy)
        const state={
            lat: this.props.store.mapGeoOffsets.lat+'',
            lon: this.props.store.mapGeoOffsets.lon+''
        };
        MAP_WS.changeMapOffset(state);
    }

    decLat() {
        const obj = this.getOffsetPureObject();
        obj.lat-=IncrementStep;
        STORE.updateMapOffsets(obj);
        this.setValues();
      }

    incLat() {
        const obj = this.getOffsetPureObject();
        obj.lat+=IncrementStep;
        STORE.updateMapOffsets(obj);
        this.setValues();
    }

    decLon() {
        const obj = this.getOffsetPureObject();
        obj.lon-=IncrementStep;
        STORE.updateMapOffsets(obj);
        this.setValues();
      }


    incLon() {
        const obj = this.getOffsetPureObject();
        obj.lon+=IncrementStep;
        STORE.updateMapOffsets(obj);
        this.setValues();
    }

    render() {
        return (
            <div className="offset-bar" >
                <div className="offset-window">
                    <div className="text">
                        <div className="label">lat: </div>
                        <input type="text" name="lat" tabIndex="1"
                    onChange={this.handleChange} value={this.props.store.mapGeoOffsets.lat} />
                        <button onClick={this.incLat}>+</button>
                        <button onClick={this.decLat}>-</button>
                    </div>
                    <div className="text">
                        <div className="label">lon:</div>
                        <input type="text" name="lon" tabIndex="2"
                    onChange={this.handleChange} value={this.props.store.mapGeoOffsets.lon} />
                        <button onClick={this.incLon}>+</button>
                        <button onClick={this.decLon}>-</button>
                    </div>
                    <button onClick={this.setValues} tabIndex="3">Set!</button>
                </div>
            </div>
        );
    }
}