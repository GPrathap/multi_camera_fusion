import React from "react";
import { MAP_WS } from "store/websocket";

export default class MapOffset extends React.PureComponent {
    constructor(){
        super();
        this.state = {
          lat: 0,
          lon: 1
        };
        this.handleChange = this.handleChange.bind(this);
    }

    handleChange (evt) {
        this.setState({ [evt.target.name]: evt.target.value });
    }

    setValues = evt => {
        MAP_WS.changeMapOffset(this.state);
    }

    render() {
        return (
            <div className="offset-bar" >
                <div className="offset-window">
                    <div className="text">
                        <div className="label">lat: </div>
                        <input type="text" name="lat"
                    onChange={this.handleChange} value={this.state.lat} />
                    </div>
                    <div className="text">
                        <div className="label">lon: </div>
                        <input type="text" name="lon"
                    onChange={this.handleChange} value={this.state.lon} />
                    </div>
                    <button onClick={this.setValues}>Set!</button>
                </div>
            </div>
        );
    }
}