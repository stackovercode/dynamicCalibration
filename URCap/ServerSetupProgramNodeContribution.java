package com.spin.dynamicCalibration.impl;

import com.ur.urcap.api.contribution.ProgramNodeContribution;
import com.ur.urcap.api.contribution.program.ProgramAPIProvider;
import com.ur.urcap.api.domain.URCapAPI;
import com.ur.urcap.api.domain.script.ScriptWriter;
import com.ur.urcap.api.domain.undoredo.UndoRedoManager;
import com.ur.urcap.api.domain.undoredo.UndoableChanges;
import com.ur.urcap.api.domain.userinteraction.keyboard.KeyboardInputCallback;
import com.ur.urcap.api.domain.userinteraction.keyboard.KeyboardInputFactory;
import com.ur.urcap.api.domain.userinteraction.keyboard.KeyboardTextInput;
import com.ur.urcap.api.domain.data.DataModel;

import com.spin.dynamicCalibration.impl.Variables;

public class ServerSetupProgramNodeContribution implements ProgramNodeContribution{
	Variables variables = new Variables();
	
	private final ProgramAPIProvider apiProvider;
	private final ServerSetupProgramNodeView view;
	private final DataModel model;
	private final UndoRedoManager undoRedoManager;
	private final KeyboardInputFactory keyboardInputFactory;

	public ServerSetupProgramNodeContribution(ProgramAPIProvider apiProvider, ServerSetupProgramNodeView view, DataModel model) {
		this.apiProvider = apiProvider;
		this.view = view;
		this.model = model;
		this.undoRedoManager = this.apiProvider.getProgramAPI().getUndoRedoManager();
		this.keyboardInputFactory = apiProvider.getUserInterfaceAPI().getUserInteraction().getKeyboardInputFactory();
	}


	@Override
	public void openView() {
		view.setJTextField(getName("IP"), "IP");		
		view.setJTextField(getName("PORT"), "PORT");		
	}


	@Override
	public void closeView() {}


	@Override
	public String getTitle() {return "Client Setup: " + getName("IP") + ", " + getName("PORT");}

	

	@Override
	public boolean isDefined() {return true;}



	@Override
	public void generateScript(ScriptWriter writer) {
		// Connect
		writer.appendLine("socketStatus = socket_open(\"" + getName("IP") + "\", " + getName("PORT") + ")");
		writer.whileCondition("socketStatus == False");
		writer.appendLine("socketStatus = socket_open(\"" + getName("IP") + "\", " + getName("PORT") + ")");
		writer.sleep(0.5);
		writer.sync();	
		writer.end();
	}


	public KeyboardTextInput getKeyboardForTextField(String textID) {
		KeyboardTextInput keyboardTextInput = keyboardInputFactory.createStringKeyboardInput();
		keyboardTextInput.setInitialValue(getName(textID));
		return keyboardTextInput;
	}


	public KeyboardInputCallback<String> getCallbackForTextField(final String jTextFieldID) {
		return new KeyboardInputCallback<String>() {
			@Override
			public void onOk(String value) {
				if ("".equals(value)) {
					if ("IP".equals(jTextFieldID)) {
						setJTextFieldValue(variables.IPv4, jTextFieldID);
						view.setJTextField(variables.IPv4, jTextFieldID);
					} else {
						setJTextFieldValue(variables.Port, jTextFieldID);
						view.setJTextField(variables.Port, jTextFieldID);
					}
				} else {
					setJTextFieldValue(value, jTextFieldID);
					view.setJTextField(value, jTextFieldID);
				}
			}
		};
	}


	private String getName(String ID) {
		if ("IP".equals(ID)) {
			return model.get(variables.IPv4, variables.IPv4);
		} else {
			return model.get(variables.Port, variables.Port);
		}
	}

	private void setJTextFieldValue(final String value, final String jTextFieldID) {
		undoRedoManager.recordChanges(new UndoableChanges() {
			@Override
			public void executeChanges() {
				if ("IP".equals(jTextFieldID)) {
					if ("".equals(value)) {
						model.remove(variables.IPv4);
					} else {
						model.set(variables.IPv4, value);
					}
				} else {
					if ("".equals(value)) {
						model.remove(variables.Port);
					} else {
						model.set(variables.Port, value);
					}
				}
			}
		});
	}
}
