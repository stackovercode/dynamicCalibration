package com.spin.dynamicCalibration.impl;

import java.awt.Component;
import java.awt.Dimension;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;

import javax.swing.Box;
import javax.swing.BoxLayout;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JTextField;

import com.ur.urcap.api.contribution.ContributionProvider;
import com.ur.urcap.api.contribution.ViewAPIProvider;
import com.ur.urcap.api.contribution.program.swing.SwingProgramNodeView;
import com.ur.urcap.api.domain.userinteraction.keyboard.KeyboardTextInput;

public class SendCMDProgramNodeView implements SwingProgramNodeView<SendCMDProgramNodeContribution>{
	private JTextField JTextFieldMessage;

	public SendCMDProgramNodeView(ViewAPIProvider apiProvider) {
		this.JTextFieldMessage = new JTextField();
	}

	@Override
	public void buildUI(JPanel panel, ContributionProvider<SendCMDProgramNodeContribution> provider) {
		panel.setLayout(new BoxLayout(panel, BoxLayout.Y_AXIS));

		panel.add(createVerticalSpacer(10));
		panel.add(createInputLine("Enter: the command: ", provider, JTextFieldMessage));
		panel.add(createVerticalSpacer(10));
	}

	private Box createDescription(final String desc) {
		Box box = Box.createHorizontalBox();
		box.setAlignmentX(Component.LEFT_ALIGNMENT);

		JLabel label = new JLabel(desc);

		box.add(label);
		return box;
	}
	
	private Box createInputLine(final String desc, final ContributionProvider<SendCMDProgramNodeContribution> provider,
			final JTextField jTextField) {
		Box inputBox = Box.createHorizontalBox();
		inputBox.setAlignmentX(Component.LEFT_ALIGNMENT);
		inputBox.add(new JLabel(desc));
		inputBox.add(createHorizontalSpacing(10));

		jTextField.setFocusable(false);
		jTextField.setPreferredSize(new Dimension(200, 24));
		jTextField.setMaximumSize(jTextField.getPreferredSize());
		jTextField.addMouseListener(new MouseAdapter() {

			@Override
			public void mousePressed(MouseEvent e) {
				if (jTextField == JTextFieldMessage) {
					KeyboardTextInput keyboardTextInput = provider.get().getKeyboardForTextField("Message");
					keyboardTextInput.show(jTextField, provider.get().getCallbackForTextField("Message"));
				}
			}
		});

		inputBox.add(jTextField);
		return inputBox;
	}

	private Component createVerticalSpacer(int height) {
		return Box.createRigidArea(new Dimension(0, height));
	}

	public void setJTextField(final String value, final String jTextFieldID) {
		if ("Message".equals(jTextFieldID)) {
			JTextFieldMessage.setText(value);
		}	
	}

	private Component createHorizontalSpacing(int horizontalSpacing) {
		return Box.createRigidArea(new Dimension(horizontalSpacing, 0));
	}
}