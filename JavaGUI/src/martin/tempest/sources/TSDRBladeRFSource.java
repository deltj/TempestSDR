/*******************************************************************************
 * Copyright (c) 2014 Martin Marinov.
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the GNU Public License v3.0
 * which accompanies this distribution, and is available at
 * http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors:
 *     Martin Marinov - initial API and implementation
 ******************************************************************************/
package martin.tempest.sources;

/**
 * Plugin for BladeRF
 * 
 * @author Ted DeLoggio
 *
 */
public class TSDRBladeRFSource extends TSDRSource {

	public TSDRBladeRFSource() {
		super("BladeRF", "TSDRPlugin_BladeRF", false);
	}

}
